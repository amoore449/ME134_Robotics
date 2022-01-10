#!/usr/bin/python
from adafruit_servokit import ServoKit
from imusensor.MPU9250 import MPU9250
import RPi.GPIO as GPIO
import time, os, sys, smbus

# Pin values
left_stepper_step = 13
left_stepper_dir = 26
right_stepper_step = 16
right_stepper_dir = 20

left_ultrasonic_trig = 5
left_ultrasonic_echo = 6
right_ultrasonic_trig = 23
right_ultrasonic_echo = 24
center_ultrasonic_trig = 17
center_ultrasonic_echo = 27

left_forward_switch = 4
left_backward_switch = 22
right_forward_switch = 25
right_backward_switch = 12

# Servo indices
right_gripper = 0
right_hook = 1
left_gripper = 15
left_hook = 14
center_gripper = 8
center_hook = 9

# Imu thresholds
x_tol = 0.5
y_tol = 0.5
z_tol = 0.5

# Setup IMU
print("Setting up IMU")
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# Class to store and move Hook
class Hook:
    kit = ServoKit(channels=16)
    # Motor values for each desired position
    open_gripper = 180
    closed_gripper = 0
    open_hook = 90
    closed_hook = 0

    def __init__(self, gripper_ind, hook_ind, trigger_pin, echo_pin):
        self.gripper = gripper_ind
        self.hook = hook_ind
        self.trigger = trigger_pin
        self.echo = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    # Get distance from ultrasonic on hook
    def get_distance(self):
        # set Trigger to HIGH
        GPIO.output(self.trigger, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)
        StartTime = time.time()
        StopTime = time.time()
        # save StartTime
        while GPIO.input(self.echo) == 0:
            StartTime = time.time()
        # save time of arrival
        while GPIO.input(self.echo) == 1:
            StopTime = time.time()
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
    
        return distance

    # Raise and close hook
    def raise_hook(self):
        # Open gripper
        self.kit.servo[self.gripper].angle = self.open_gripper
        time.sleep(1)
        self.kit.servo[self.gripper].angle = self.closed_gripper / 2
        time.sleep(0.1)
        # Raise hook
        self.kit.servo[self.hook].angle = self.closed_hook
        time.sleep(0.1)
        # Close gripper
        self.kit.servo[self.gripper].angle = self.closed_gripper
        time.sleep(0.1)
    
    # Open and lower hook
    def lower_hook(self):
        # Open gripper
        self.kit.servo[self.gripper].angle = self.open_gripper / 2
        time.sleep(0.1)
        # Lower hook
        self.kit.servo[self.hook].angle = self.open_hook
        time.sleep(0.1)
        self.kit.servo[self.gripper].angle = self.open_gripper
        time.sleep(0.1)


# Class to store and manipulate arms
class Arm:
    arm_frequency = 2000 

    def __init__(self, right, gripper_ind, hook_ind, trigger_pin, echo_pin, step_pin, dir_pin, front_switch_pin, back_switch_pin):
        self.right = right
        self.hook = Hook(gripper_ind, hook_ind, trigger_pin, echo_pin)
        self.step = step_pin
        self.dir = dir_pin
        self.front_switch = front_switch_pin
        self.back_switch = back_switch_pin
        # Setup stepper
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.step, GPIO.OUT)
        GPIO.setup(self.dir, GPIO.OUT)
        GPIO.output(self.step, GPIO.HIGH)
        self.p = GPIO.PWM(self.step, 5000)
        #Setup limit switch
        GPIO.setup(self.front_switch, GPIO.IN)
        GPIO.setup(self.back_switch, GPIO.IN)

    # Release and move arm forward individually until hit switch or next beam
    def move_arm(self):
        # Release hook from beam
        self.hook.lower_hook()
        # Stepper setup
        self.p.ChangeFrequency(self.arm_frequency)
        GPIO.output(self.dir, not self.right)
        print("Setup done")
        # Extend arm until beam detected or switch triggered
        ultrasonic_limit = 10
        distance = self.hook.get_distance()
        print(distance)
        while (distance > ultrasonic_limit and not GPIO.input(self.back_switch)): 
            self.p.start(1)
            time.sleep(0.01)
            distance = self.hook.get_distance()
            print("Distance: ", distance)
            print("Switched pushed: ", GPIO.input(self.back_switch))
        self.hook.raise_hook()
    
    # Release pins
    def cleanup(self):
        self.p.stop()
        GPIO.cleanup()

# Determine if body is stable with IMU
def is_stable():
    imu.readSensor()
    imu.computeOrientation()

    print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
    x_accel = imu.AccelVals[0]
    y_accel = imu.AccelVals[1]
    z_accel = imu.AccelVals[2]

    if (abs(x_accel) > x_tol and abs(y_accel) > y_tol and abs(z_accel) > z_tol):
        return False

    return True

# Move body from one rung to another by moving both arms backwards
def move_body(right_arm, left_arm, body_hook):
    body_hook.lower_hook()
    # Setup arm steppers
    right_arm.p.ChangeFrequency(1000)
    left_arm.p.ChangeFrequency(1000)
    GPIO.output(right_arm.dir, right_arm.right)
    GPIO.output(left_arm.dir, left_arm.right)

    # Move until reach next rung or hit either front limit switch
    ultrasonic_limit = 10
    distance = body_hook.get_distance()
    print(distance)
    while (distance > ultrasonic_limit and not (GPIO.input(right_arm.front_switch) or GPIO.input(left_arm.front_switch))): 
        right_arm.p.start(1)
        left_arm.p.start(1)
        time.sleep(0.01)
        distance = body_hook.get_distance()
        print("Distance: ", distance)
        print("Right switched pushed: ", GPIO.input(right_arm.front_switch))
        print("Left switched pushed: ", GPIO.input(left_arm.front_switch))
    
    body_hook.raise_hook()    

# Set up main objects
print("Start")
right = Arm(True, right_gripper, right_hook, right_ultrasonic_trig, right_ultrasonic_echo, right_stepper_step, right_stepper_dir, right_forward_switch, right_backward_switch)
left = Arm(False, left_gripper, left_hook, left_ultrasonic_trig, left_ultrasonic_echo, left_stepper_step, left_stepper_dir, left_forward_switch, left_backward_switch)
body_hook = Hook(center_gripper, center_hook, center_ultrasonic_trig, center_ultrasonic_echo)

# Initial grab
right.hook.raise_hook()
left.hook.raise_hook()
body_hook.raise_hook()
time.sleep(2)

# Main loop
try: 
    while (True):
        # Extend right arm
        right.move_arm()
        # Wait for body to stabilize
        while (not is_stable()):
            time.sleep(0.5)
        # Move body from one rung to next
        move_body(right, left, body_hook)
        # Wait for body to stabilize
        while (not is_stable()):
            time.sleep(0.5)
        # Retract left arm to catch up
        left.move_arm()
        # Wait for body to stabilize
        while (not is_stable()):
            time.sleep(0.5)   
except KeyboardInterrupt:
    # Unhook from beam and clean up
    right.hook.lower_hook()
    left.hook.lower_hook()
    body_hook.lower_hook()

    right.cleanup()
    left.cleanup()
