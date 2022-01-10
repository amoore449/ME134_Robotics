#!/usr/bin/env/python3.7
import time, math, sys
import board, busio
import RPi.GPIO as GPIO
from surface_mapping import *
from adafruit_servokit import ServoKit
import adafruit_vl53l0x
from motors import IK_Solve
from get_xy import get_xy, read_SVG
from svgpathtools import svg2paths2 

#Initialize servo communication
kit = ServoKit(channels=16)

#Setup GPIO control
LED_RED = 23
LED_GREEN = 24
MESH_PIN = 27
DRAW_PIN = 17

GPIO.setmode(GPIO.BCM)

GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)
GPIO.setup(MESH_PIN, GPIO.IN)
GPIO.setup(DRAW_PIN, GPIO.IN)

#Start communication with Lidar
i2c = busio.I2C(board.SCL, board.SDA)
lidar = adafruit_vl53l0x.VL53L0X(i2c)
lidar.measurement_timing_budget = 50000

#Move servos to initial positions
kit.servo[0].angle = 90
kit.servo[1].angle = 120
kit.servo[2].angle = 150
kit.servo[3].angle = 90

#Read in SVG file
xy_coords = read_SVG("Examples/initials_JA.svg", 10)

def time_ms(): #Convert from ns to ms
    return time.time_ns() / 1000000

def main():
    #Initialize global state variables
    previous_time = 0
    isMoving = False
    hasZMesh = False
    sample_z = False
    isWriting = False
    z_offset = 20 #Distance from base frame to floor in mm
    N = 20
    step_interval = 100 #time per motor move in ms
    prev_state = 0
    i = 0

    while True:
        try:
            current_time = time_ms()
            sample_z = GPIO.input(MESH_PIN)
            isWriting = GPIO.input(DRAW_PIN)

            if sample_z == True:
                GPIO.output(LED_RED, 1)
                z_data = sample_surface(N, (0, 200), (10, 100))
                inter, coef = surface_approximation(N, z_data, (0, 200), (10, 100))
                hasZMesh = True
            else:
                GPIO.output(LED_RED, 0)
                hasZMesh = False

            if hasZMesh: #Check for z mesh
                GPIO.output(LED_GREEN, 1)
            else: GPIO.output(LED_GREEN, 0)

            if (isWriting == True) & (not isMoving):
                if current_time - previous_time >= step_interval: #Check to see if we have taken longer than the step interval
                    # Get x,y coordinates for drawing
                    returned = get_xy(xy_coords, i)
                    if returned == None:
                        break
                    else:
                        x = returned[0]
                        y = returned[1]
                        down = returned[2]


                    #Get Z given current z mesh status
                    if hasZMesh:
                        z = fit_func((x, y), inter, coef) #Find z point given x and y coordinates
                    else:
                        z = z_offset #Otherwise write on a flat plane

                    previous_time = current_time

                    #Attempt IK Solving
                    try:
                        angles = IK_Solve(x, y, z) #Try to solve IK equations
                    except Exception as e:
                        print("Inverse Kinematics failed! Exception: ", e)

                    #Move the servos 
                    for j in range(len(angles)):
                        kit.servo[j].angle = angles[j]
                    i += 1

                    #Move pen up or down 
                    if (down and not prev_state):
                        pendown(x, y, z)
                    elif (not down and prev_state):
                        penup(x, y, z)

                    previous_time += step_interval #Make sure the timing is consistent

        except KeyboardInterrupt:
            break
    return

def sample_z_point(x, y): #Sample a z point with lidar sensor
    angles = IK_Solve(x, y, 0)

    for j in range(len(angles)):
        kit.servo[j].angle = angles[j]
    time.sleep(0.5)
    return lidar.range

def sample_surface(N, xbounds, ybounds): #Sample points in N x N grid
    z_data = np.empty((N,N))

    xmin = xbounds[0]
    xmax = xbounds[1]

    ymin = ybounds[0]
    ymax = ybounds[1]

    x = np.linspace(xmin, xmax, N - 1)
    y = np.linspace(ymin, ymax, N - 1)
    i = 0
    j = 0
    for xval in x:
        for yval in y:
            zval = sample_z_point(xval, yval)
            z_data[i][j]
            j += 1
        i += 1

def pendown(x, y, z):
    prev_state = 1
    #Attempt IK Solving
    try:
        angles = IK_Solve(x, y, z-10)
    except Exception as e:
        print("Inverse Kinematics failed! Exception: ", e)
    for j in range(len(angles)):
        kit.servo[j].angle = angles[j]

def penup(x, y, z):
    prev_state = 0
    try:
        angles = IK_Solve(x, y, z+10)
    except Exception as e:
        print("Inverse Kinematics failed! Exception: ", e)
    for j in range(len(angles)):
        kit.servo[j].angle = angles[j]

if __name__ == "__main__":
    main()
