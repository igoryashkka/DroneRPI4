#Include the library files
import RPi.GPIO as GPIO
import smbus 
from time import sleep

#from PID import PID 

#ports num
#pwmpin1 = 16
#pwmpin2 = 36
#pwmpin3 = 38
#pwmpin4 = 40
pwmpin1 = 12 #front-rigt-ccw
pwmpin2 = 32 #rear-right -cw
pwmpin3 = 33 #rear-left-ccw
pwmpin4 = 35 #front - left -cw 



#sets
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

#configuring ports for out
GPIO.setup(pwmpin1, GPIO.OUT)
GPIO.setup(pwmpin2, GPIO.OUT)
GPIO.setup(pwmpin3, GPIO.OUT)
GPIO.setup(pwmpin4, GPIO.OUT)

#configuring ports for PWM for 50 Herz
pi_pwm1 = GPIO.PWM(pwmpin1, 50)
pi_pwm2 = GPIO.PWM(pwmpin2, 50)
pi_pwm3 = GPIO.PWM(pwmpin3, 50)
pi_pwm4 = GPIO.PWM(pwmpin4, 50)
#sleep(3)
#init motors
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

#sleep(5)
print("END init")

avr_throttle = 8.0

pid_output_pitch = 0.0
#pi_pwm1.ChangeDutyCycle(5)
#pi_pwm2.ChangeDutyCycle(5)
#pi_pwm3.ChangeDutyCycle(5)
#pi_pwm4.ChangeDutyCycle(5)
pid_output_roll = 0.0


#front-right  =  CCW
esc_1 = avr_throttle - pid_output_pitch + pid_output_roll #front-rigt-ccw
esc_2 = avr_throttle + pid_output_pitch + pid_output_roll #rear-right -cw
esc_3 = avr_throttle + pid_output_pitch - pid_output_roll #rear-left-ccw
eac_4 = avr_throttle - pid_output_pitch - pid_output_roll #front - left -cw 
# Setup GPIO pins
#GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Set the servo motor pin as output pin
#GPIO.setup(4,GPIO.OUT)

#pwm = GPIO.PWM(4,50)
#pwm.start(0)

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47



integral_ = 0.0
prev_err = 0.0


bus = smbus.SMBus(1)# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68 # MPU6050 device address

#def angle(Angle):
 #   duty = Angle / 18 + 2
  #  GPIO.output(7,True)
   # pwm.ChangeDutyCycle(duty)
    # sleep(1)
   # GPIO.output(7,False)
    # pwm.ChangeDutyCycle(0)
    
#def setAngle():
 #   angle(90)

def MPU_Init():
    
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def calc_pid(input_angle, set_angle,Kp, Ki,Kd, dt):
	err = set_angle - input_angle
	integral_ = 0.0
	prev_err = 0.0
	integral_ += err * dt
	D = (err - prev_err) / dt
	prev_err = err
	return (err * Kp + integral_ * Ki + D * Kd)



def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


def scaled_value(value,old_min,old_max,new_min,new_max):
        new_value = (value - old_min)*(new_max - new_min)/(old_max - old_min) + new_min
        return new_value
    

MPU_Init()
#pwm.ChangeDutyCycle(10)
#sleep(3)
#pwm.ChangeDutyCycle(5)
#sleep(3)
#pwm.ChangeDutyCycle(7)
while True:


    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT)
    acc_y = read_raw_data(ACCEL_YOUT)
    acc_z = read_raw_data(ACCEL_ZOUT)

    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT)
    gyro_y = read_raw_data(GYRO_YOUT)
    gyro_z = read_raw_data(GYRO_ZOUT)

    Ax = acc_x/16384.0
    Ay = acc_y/16384.0 
    Az = acc_z/16384.0

    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0

# Uncomment below line to see the Accelerometer and Gyroscope values   
#    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)  
       
    in_min = 1
    in_max = -1
    out_min = 0
    out_max = 180
    
 #   setAngle() # Use this function to set the servo motor point
    
    # Convert accelerometer Y axis values from 0 to 180   
    value = (Ay - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    value = int(value)
    value_x  = (Ax - in_min) * (out_max - out_min) / (in_max - in_min) + out_min    
    value_x = int(value_x)
    #print("x = ", value_x)  
    #print("y = ",value)
   # if value >= 0 and value <= 180:
        # Write these values on the servo motor
#         angle(value) # Rotate the servo motor using the sensor values
    sleep(0.08)

    pid_output_pitch = calc_pid(value_x,90, 1.0,2.0,3.0,0.08)
    pid_output_roll = calc_pid(value,90, 1.0,2.0,3.0,0.08)
    #print("pith :  ")
    #print(pid_output_pitch)
    #print("roll : ")
    #print(pid_output_roll)
    #pid_output_roll  = calc_pid()


    esc_1 = avr_throttle - pid_output_pitch + pid_output_roll
    # pi_pwm1.ChangeDutyCycle(esc_1)

    esc_2 = avr_throttle + pid_output_pitch + pid_output_roll
    #pi_pwm2.ChangeDutyCycle(esc_2)



    esc_3 = avr_throttle + pid_output_pitch - pid_output_roll
    #pi_pwm2.ChangeDutyCycle(esc_3)



    esc_4 = avr_throttle - pid_output_pitch - pid_output_roll
    #pi_pwm2.ChangeDutyCycle(esc_4)
    
    esc_1 = scaled_value(esc_1,-4000,4000,6,10)
    esc_2 = scaled_value(esc_2,-4000,4000,6,10)
    esc_3 = scaled_value(esc_3,-4000,4000,6,10)
    esc_4 = scaled_value(esc_4,-4000,4000,6,10)
    pi_pwm1.ChangeDutyCycle(esc_1)
    pi_pwm2.ChangeDutyCycle(esc_2)
    pi_pwm2.ChangeDutyCycle(esc_3)
    pi_pwm2.ChangeDutyCycle(esc_4)
    
    print("esc_1 ", esc_1) 
    print("esc_2 ", esc_2) 
    print("esc_3 ", esc_3) 
    print("esc_4 ", esc_4)  
    #print("y = ",value)






