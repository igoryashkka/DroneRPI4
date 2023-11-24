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


factor = -0.05


def scalling_thread():
    # if factor < 0.75:
    global factor
    factor = factor + 0.001
    print(factor)
    return factor
   # print("Function called at:", time.strftime("%Y-%m-%d %H:%M:%S"))


def scheduler():
    while True:
        scalling_thread()
        time.sleep(0.6)


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

def scaled_value_(value, old_min, old_max, new_min, new_max, scaling_factor=1.0):
    new_value = (value - old_min) * (new_max - new_min) / \
        (old_max - old_min) + new_min
    scaled_new_value = new_value +  new_value * scaling_factor
    return scaled_new_value

def scaled_value(value, old_min, old_max, new_min, new_max):
    new_value = (value - old_min)*(new_max - new_min) / \
        (old_max - old_min) + new_min
    return new_value


def scaled_value_factor(value, old_min, old_max, new_min, new_max, speed_factor=1.0):
    new_value_scaled = (value - old_min)*(new_max - new_min) / \
        (old_max - old_min) + new_min

    return new_value_scaled + speed_factor

# -------------------------- PID SECTION -------------------------- #

#duty_cycle_values = {'pi_pwm1': 0 , 'pi_pwm2': 0,'pi_pwm3': 0,'pi_pwm4': 0}

def motors_cdc(duty_cycle_value):
    pi_pwm1.ChangeDutyCycle(duty_cycle_value['pi_pwm1'])
    pi_pwm2.ChangeDutyCycle(duty_cycle_value['pi_pwm2'])
    pi_pwm3.ChangeDutyCycle(duty_cycle_value['pi_pwm3'])
    pi_pwm4.ChangeDutyCycle(duty_cycle_value['pi_pwm4'])
    #print("Current duty cycle is: ", duty_cycle_value)



def motors_cdc_esc(esc_values):
    pi_pwm1.ChangeDutyCycle( esc_values['esc_1'])
    pi_pwm2.ChangeDutyCycle( esc_values['esc_2'])
    pi_pwm2.ChangeDutyCycle( esc_values['esc_3'])
    pi_pwm2.ChangeDutyCycle( esc_values['esc_4'])

    #print("Current duty cycle is: ", esc_values)




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


def motors_init(duty_cycle_values):
    duty_cycle_values = {'pi_pwm1': 10 , 'pi_pwm2': 10,'pi_pwm3': 10,'pi_pwm4': 10}
    print("init")
    motors_cdc_esc(duty_cycle_values)

    sleep(2)
    duty_cycle_values = {'pi_pwm1': 5 , 'pi_pwm2': 5,'pi_pwm3': 5,'pi_pwm4': 5}
    motors_cdc_esc(duty_cycle_values)

    sleep(2)
    duty_cycle_values = {'pi_pwm1': 8 , 'pi_pwm2': 8,'pi_pwm3': 8,'pi_pwm4': 8}
    motors_cdc_esc(duty_cycle_values)

    sleep(3)
    print("END init")

# Change duty cycle of motors


###########################################################
avr_throttle = 7.5


def reading_raw_acc(dict_acc):
    dict_acc['x'] =  read_raw_data(ACCEL_XOUT)
    dict_acc['y'] =  read_raw_data(ACCEL_YOUT)
    dict_acc['z'] =  read_raw_data(ACCEL_ZOUT)

    return dict_acc


def reading_raw_gyro(dict_gyro):
    dict_gyro['x'] =  read_raw_data(GYRO_XOUT)
    dict_gyro['y'] =  read_raw_data(GYRO_YOUT)
    dict_gyro['z'] =  read_raw_data(GYRO_ZOUT)

    return dict_gyro
 
def conversation_data(dict_acc,dict_gyro,dict_angles):
    Ax = dict_acc['x']/16384.0
    Ay = dict_acc['y']/16384.0
    Az = dict_acc['z']/16384.0

    Gx = dict_gyro['x']/131.0
    Gy = dict_gyro['y']/131.0
    Gz = dict_gyro['z']/131.0

    in_min = 1
    in_max = -1
    out_min = 0
    out_max = 180

    value_y = (Ay - in_min) * (out_max - out_min) / \
                (in_max - in_min) + out_min

    dict_angles['x_angel'] = int(value_y)

    value_x = (Ax - in_min) * (out_max - out_min) / \
                (in_max - in_min) + out_min
    
    dict_angles['y_angel']  = int(value_x)
    
    return dict_angles



def esc_calc(esc_values,pids_output): 
    esc_values['esc_1'] = avr_throttle - pids_output['pid_output_pitch'] + pids_output['pid_output_roll']
    esc_values['esc_2'] = avr_throttle + pids_output['pid_output_pitch'] + pids_output['pid_output_roll']
    esc_values['esc_3'] = avr_throttle + pids_output['pid_output_pitch'] - pids_output['pid_output_roll']
    esc_values['esc_4'] = avr_throttle - pids_output['pid_output_pitch'] - pids_output['pid_output_roll']

def debug_print_esc(esc_values):
    print("------------")
    print( esc_values['esc_1'])
    print( esc_values['esc_2'])
    print( esc_values['esc_3'])
    print( esc_values['esc_4'])
    print("------------")

##########
#Main point
##########
if __name__ == "__main__":
########################################################
#Declaration phase
########################################################

        dict_acc = {'x':0,'y':0,'z':0}
        dict_gyro = {'x':0,'y':0,'z':0}
        dict_angles = {'x_angel':0, 'y_angel':0}
        time_sleep = 0.08

        esc_values = {'esx_1':0,'esx_2':0,'esx_3':0,'esc_4':0 }

        target_angel_x = 90
        target_angel_y = 90

        Kp = 1.0;Ki = 2.0 ;Kd = 3.0

        pids_output = {'pid_output_pitch':0, 'pid_output_roll':0}
        duty_cycle_values = {'pi_pwm1': 0 , 'pi_pwm2': 0,'pi_pwm3': 0,'pi_pwm4': 0}

        esc_min_value = -5000; esc_max_value = 5000
        scaled_esc_min_value = 5; scaled_esc_max_value = 10

########################################################
#Initialization phase
########################################################

        MPU_Init()
        motors_init(duty_cycle_values)

        sleep(2)
        

        scheduler_thread = threading.Thread(target=scheduler)
        scheduler_thread.start()
########################################################
#Endles-loop phase
########################################################       
        while True:

            dict_acc = reading_raw_acc(dict_acc)
            dict_gyro = reading_raw_acc(dict_gyro)
            dict_angles = conversation_data(dict_angles)

        
            sleep(time_sleep)

            pids_output['pid_output_pitch'] = calc_pid(dict_angles['x_angel'], target_angel_x, Kp, Ki, Kd, time_sleep)
            pids_output['pid_output_roll']  = calc_pid(dict_angles['y_angel'], target_angel_y, Kp, Ki, Kd, time_sleep)

            esc_values = esc_calc(esc_values,pids_output)
	    
            esc_values['esc_1'] = scaled_value_(esc_values['esc_1'], esc_min_value, esc_max_value, scaled_esc_min_value, scaled_esc_max_value, factor)
            esc_values['esc_2'] = scaled_value_(esc_values['esc_2'], esc_min_value, esc_max_value, scaled_esc_min_value, scaled_esc_max_value, factor)
            esc_values['esc_3'] = scaled_value_(esc_values['esc_3'], esc_min_value, esc_max_value, scaled_esc_min_value, scaled_esc_max_value, factor)
            esc_values['esc_4'] = scaled_value_(esc_values['esc_4'], esc_min_value, esc_max_value, scaled_esc_min_value, scaled_esc_max_value, factor)

            debug_print_esc(esc_values)

            #duty_cycle_values
            motors_cdc_esc(esc_values)
            

           

   # else:
    #    print("Motors are not initialized")
