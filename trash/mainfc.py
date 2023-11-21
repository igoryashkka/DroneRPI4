#      MAIN TEST FILE
#      created by: Zozulia Dmytro 2023
#      Description: This file contains motors initialization from main file and tests with different duty cycles ftom 5 to 10
#      to understandn the values of PWM when motors start to lift drone up, make it hover in one place, softly land and standby

# imports
import RPi.GPIO as GPIO
import smbus
from time import sleep

import threading
import time


factor = 0


def my_function():
    if factor < 0.75:
 `       factor = factor + 0.05

    print("Function called at:", time.strftime("%Y-%m-%d %H:%M:%S"))


def scheduler():
    while True:
        my_function()
        time.sleep(1)


# -------------------------- MPU SECTION -------------------------- #
##
# MPU6050 Registers&Address
##

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards

Device_Address = 0x68  # MPU6050 device address
# -------------------------- MPU SECTION -------------------------- #
##
# Init MPU
##


def MPU_Init():

    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value


# -------------------------- MPU SECTION -------------------------- #

# -------------------------- PID SECTION -------------------------- #
integral_ = 0.0
prev_err = 0.0
##
# calc_pid function for calculaction PID output value
##


def calc_pid(input_angle, set_angle, Kp, Ki, Kd, dt):
    err = set_angle - input_angle
    integral_ = 0.0
    prev_err = 0.0
    integral_ += err * dt
    D = (err - prev_err) / dt
    prev_err = err
    return (err * Kp + integral_ * Ki + D * Kd)
##
# scaled_value function for adjusting esc-raw values
##


def scaled_value(value, old_min, old_max, new_min, new_max):
    new_value = (value - old_min)*(new_max - new_min) / \
        (old_max - old_min) + new_min
    return new_value


def scaled_value_factor(value, old_min, old_max, new_min, new_max, speed_factor=1.0):
    new_value_scaled = (value - old_min)*(new_max - new_min) / \
        (old_max - old_min) + new_min

    return (new_value_scaled + (old_max - new_value_scaled) * (1.0 - speed_factor))

# -------------------------- PID SECTION -------------------------- #


# pins
pwmpin1 = 12  # front-rigt-ccw
pwmpin2 = 32  # rear-right -cw
pwmpin3 = 33  # rear-left-ccw
pwmpin4 = 35  # front - left -cw

# sets
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# configuring ports for out
GPIO.setup(pwmpin1, GPIO.OUT)
GPIO.setup(pwmpin2, GPIO.OUT)
GPIO.setup(pwmpin3, GPIO.OUT)
GPIO.setup(pwmpin4, GPIO.OUT)

# configuring ports for PWM for 50 Herz
pi_pwm1 = GPIO.PWM(pwmpin1, 50)
pi_pwm2 = GPIO.PWM(pwmpin2, 50)
pi_pwm3 = GPIO.PWM(pwmpin3, 50)
pi_pwm4 = GPIO.PWM(pwmpin4, 50)

# init motors


def motors_init():

    print("init")
    pi_pwm1.start(10)
    pi_pwm2.start(10)
    pi_pwm3.start(10)
    pi_pwm4.start(10)

    sleep(2)

    pi_pwm1.ChangeDutyCycle(5)
    pi_pwm2.ChangeDutyCycle(5)
    pi_pwm3.ChangeDutyCycle(5)
    pi_pwm4.ChangeDutyCycle(5)

    sleep(2)

    pi_pwm1.ChangeDutyCycle(8)
    pi_pwm2.ChangeDutyCycle(8)
    pi_pwm3.ChangeDutyCycle(8)
    pi_pwm4.ChangeDutyCycle(8)

    sleep(3)
    print("END init")

# Change duty cycle of motors


def motors_cdc(duty_cycle_value):

    pi_pwm1.ChangeDutyCycle(duty_cycle_value)
    pi_pwm2.ChangeDutyCycle(duty_cycle_value)
    pi_pwm3.ChangeDutyCycle(duty_cycle_value)
    pi_pwm4.ChangeDutyCycle(duty_cycle_value)
    print("Current duty cycle is: ", duty_cycle_value)


# main
if __name__ == "__main__":

    MPU_Init()

    print("Mpu init - ok \n\n")
    print("Duty Cycle Test \n\n")
    print("Init motors? y/n")
    answer = input(">>> ")
    if answer == "y":

        motors_init()
        sleep(2)
        print("Motors are initialized")

        scheduler_thread = threading.Thread(target=scheduler)
        scheduler_thread.start()

        while True:

            acc_x = read_raw_data(ACCEL_XOUT)
            acc_y = read_raw_data(ACCEL_YOUT)
            acc_z = read_raw_data(ACCEL_ZOUT)

            # Read Gyroscope raw value
            gyro_x = read_raw_data(GYRO_XOUT)
            gyro_y = read_raw_data(GYRO_YOUT)
            gyro_z = read_raw_data(GYRO_ZOUT)

            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0

            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0

            in_min = 1
            in_max = -1
            out_min = 0
            out_max = 180

            # Convert accelerometer Y axis values from 0 to 180
            value = (Ay - in_min) * (out_max - out_min) / \
                (in_max - in_min) + out_min

            value = int(value)
            value_x = (Ax - in_min) * (out_max - out_min) / \
                (in_max - in_min) + out_min
            value_x = int(value_x)
            #         angle(value) # Rotate the servo motor using the sensor values
            sleep(0.08)

            pid_output_pitch = calc_pid(value_x, 90, 1.0, 2.0, 3.0, 0.08)
            pid_output_roll = calc_pid(value, 90, 1.0, 2.0, 3.0, 0.08)

            esc_1 = 7.5 - pid_output_pitch + pid_output_roll

            esc_2 = 7.5 + pid_output_pitch + pid_output_roll

            esc_3 = 7.5 + pid_output_pitch - pid_output_roll

            esc_4 = 7.5 - pid_output_pitch - pid_output_roll

            esc_1 = scaled_value_factor(esc_1, -5000, 5000, 5, 10, factor)
            esc_2 = scaled_value_factor(esc_2, -5000, 5000, 5, 10, factor)
            esc_3 = scaled_value_factor(esc_3, -5000, 5000, 5, 10, factor)
            esc_4 = scaled_value_factor(esc_4, -5000, 5000, 5, 10, factor)
            pi_pwm1.ChangeDutyCycle(5.3)
            pi_pwm2.ChangeDutyCycle(5.3)
            pi_pwm2.ChangeDutyCycle(5.3)
            pi_pwm2.ChangeDutyCycle(5.3)

            # print("Enter duty cycle value: ")
            # duty_cycle_value = float(input(">>> "))
            # motors_cdc(duty_cycle_value)

    else:
        print("Motors are not initialized")
