import RPi.GPIO as GPIO
from time import sleep

class Stepper():
    def __init__(self, channel):
        GPIO.setmode(GPIO.BOARD)

        self.motor_channel = channel
        self.out = (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH)
        self.angle = 0
        self.steps = 0
        GPIO.setup(self.motor_channel, GPIO.OUT)

    def step(self, clockwise):
        if clockwise:
            # First, shift GPIO outputs to step in appropriate direction
            self.out = (self.out[3], self.out[0], self.out[1], self.out[2])

            GPIO.output(self.motor_channel, self.out)
        else: # counter-clockwise
            # First, shift GPIO outputs to step in appropriate direction
            self.out = (self.out[1], self.out[2], self.out[3], self.out[0])

            GPIO.output(self.motor_channel, self.out)

    # Function to test ability to rotate given angles
    def rotate_angle(self, angle):
        direction = (angle == abs(angle)) # True if positive, False if negative
        steps = int(angle / 360.0 * 200)

        for i in range(abs(steps)):
            self.step(direction)
            sleep(0.02)

        self.angle += angle
        self.steps += steps

    # Function to rotate motor to absolute steps location
    def rotate_to_steps(self, steps):
        diff = steps - self.steps
        direction = (diff == abs(diff))

        for i in range(abs(diff)): # step appropriate num steps
            self.step(direction)
            sleep(0.03)

        # Update attributes
        self.steps = steps
        self.angle = self.angle + (diff * 360.0 / 200)

    # Function to reset GPIO pins
    def off(self):
        self.out = (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW)
        GPIO.output(self.motor_channel, self.out)
        GPIO.cleanup()

    def get_angle(self):
        print(self.angle)

    def get_steps(self):
        print(self.steps)