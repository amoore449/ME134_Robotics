
#HW1 Distance Sensor
#Allison Moore



import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

#Servo Pins
servoPIN = 17
servoPIN2 = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
GPIO.setup(servoPIN2, GPIO.OUT)

#setAngle
#sets servo angle
#modified from: https://learn.sparkfun.com/tutorials/raspberry-gpio/python-rpigpio-api
def setAngle(angle, pwm, myservoPIN):
    duty = angle / 18 + 3
    GPIO.output(myservoPIN, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(myservoPIN, False)
    pwm.ChangeDutyCycle(duty)

#turn_page
#runs two servos to turn page
def turn_page():
    p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
    p2 = GPIO.PWM(servoPIN2, 50) # GPIO 17 for PWM with 50Hz
    p.start(2.5) # Initialization
    setAngle(0, p, servoPIN)
    time.sleep(.5)
    setAngle(150, p, servoPIN)
    time.sleep(1)
    setAngle(270, p, servoPIN)
    time.sleep(1)

    p2.start(2.5) # Initialization
    setAngle(0,p2,servoPIN2)
    time.sleep(.5)
    setAngle(30,p2,servoPIN2)
    time.sleep(.5)
    setAngle(70,p2,servoPIN2)
    time.sleep(.5)
    setAngle(130,p2,servoPIN2)
    time.sleep(.5)
    setAngle(180,p2,servoPIN2)
    time.sleep(1)

#Distance
#Measures distance
#Referenced from: https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

if __name__ == '__main__':
    try:
        while True:
            dist = distance()
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(1)
            if dist < 10:
                turn_page()

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
