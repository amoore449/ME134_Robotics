from Stepper import *
import sys
import datetime

#assign GPIO pins for motor
motor_channel = (29,31,33,35)  
stepper = Stepper(motor_channel)

time = datetime.datetime.now().time()
time.strftime("%I:%M %p")

print("Hour: " + str(time.hour))
print("Minute: " + str(time.minute))

# Dictionary to map hour to stepper location
hours_mapper = {
    1 : 0,
    2 : 260,
    3 : 480,
    4 : 680,
    5 : 920,
    6 : 1140,
    7 : 1360,
    8 : 1610,
    9 : 1820,
    10 : 2080,
    11 : 2320,
    12 : 2560
}

# Dictionary to map minute to servo location
minutes_mapper = {
    0 : 6,
    5 : 5.8,
    10 : 5.5,
    15 : 5,
    20 : 4.5,
    25 : 4.2,
    30 : 3.8,
    35 : 3.4,
    40 : 3.1,
    45 : 2.7,
    50 : 2.4,
    55 : 2.2,
    60 : 2.1
}

# Setup servo:
servo_pin = 32
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

servo_pwm = GPIO.PWM(servo_pin, 50)
servo_pwm.start(0)

# Function for rotating servo to absolute position
def rotate_servo(duty):
    GPIO.output(servo_pin, True)
    servo_pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(servo_pin, False)
    servo_pwm.ChangeDutyCycle(duty)


minute = time.minute - 1

while True:
    try:
        time = datetime.datetime.now().time()
        time.strftime("%I:%M %p")

        while(time.minute == minute): # wait until a minute has passed
             sleep(1) # wait for 1 second to pass since last iteration
             time = datetime.datetime.now().time()
             time.strftime("%I:%M %p")

        hour = time.hour
        minute = time.minute

        if hour > 12: # convert to 12 hour
            hour -= 12

        # Linear approximation for minutes hand location
        lower_5 = rounded = int(minute) - int(minute) % 5
        upper_5 = lower_5 + 5
        slope = (minutes_mapper[upper_5] - minutes_mapper[lower_5]) / 5
        angle = minutes_mapper[lower_5] + slope * (minute - lower_5) # linear fit to map angle

        rotate_servo(angle)
        stepper.rotate_to_steps(hours_mapper[hour])

    except KeyboardInterrupt:
        stepper.off()
