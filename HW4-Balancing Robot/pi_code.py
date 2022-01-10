from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)
motor = 0
STOP = -0.1
angle = 0

kit.continuous_servo[motor].throttle = STOP
time.sleep(1)

# Rotate rod positive 90 degrees
def forward_90():
    kit.continuous_servo[motor].throttle = -1
    time.sleep(2.1)
    kit.continuous_servo[motor].throttle = STOP

# Rotate rod positive 90 degrees
def backward_90(): 
    kit.continuous_servo[motor].throttle = 1
    time.sleep(2.1)
    kit.continuous_servo[motor].throttle = STOP

# Mechanism to reset motor counterclockwise
def rotate_counterclockwise():
    forward_90()
    #Pause for balancing to start
    time.sleep(5)
    backward_90()

# Mechanism to reset motor counterclockwise
def rotate_clockwise():
    backward_90()    
    #Pause for balancing to start
    time.sleep(5)
    forward_90()

# Rotate clockwise if rod falls more than -30degrees, counterclockwise if more than +30
if (angle < -30):
    rotate_clockwise()
elif (angle > 30):
    rotate_counterclockwise

time.sleep(10)
