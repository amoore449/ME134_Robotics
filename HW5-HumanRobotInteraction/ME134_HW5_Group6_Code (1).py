'''
ME134 HOMEWORK 5 - HRI ROBOT - CODE 
Due: November 23rd, 2021
Group: Allison Moore, Olive Rappoli, Rebecca Shen 

How It Works:

1. Robot base rotates and uses ultrasonic sensor to find objects in range (>60cm but less than 200cm)
to filter out objects that might not be people. The corresponding motor angles are stored in an array.
2. Robot moves to first angle in array and checks for a face. If a face is detected, it tracks it until 
the face is in the center of the frame. Then, finger detection is run. Once the user puts up all 10 fingers, 
the ball launches.
3. The ball is launched by running the motors for a few seconds, and then lifting up the trap door. Then the 
motors stop, and the trap door is put back down. 
4. The robot stays in place and waits for the ball to be thrown back, indicated by the state of the limit switch. 
5. Once the ball is thrown back, the robot moves to the next angle in the array, and if there is no face, it sweeps 
back and forth until it finds it.
'''

#---------------------------------------------
# SETUP
#---------------------------------------------

### IMPORTING THINGS ###

# camera #
import imutils
import cv2
from imutils.video import VideoStream
from sklearn.metrics import pairwise
import numpy as np
# servo #
from adafruit_servokit import ServoKit
# other #
import time
import math
import RPi.GPIO as GPIO

### SERVO MOTOR SETUP ###
kit = ServoKit(channels=16)
baseMotor = kit.servo[3]
trapDoor = kit.servo[12]

### CAMERA SETUP ###
faceCascade = cv2.CascadeClassifier('/home/pi/Desktop/haarcascade_frontalface_default.xml')
eyeCascade = cv2.CascadeClassifier('/home/pi/Desktop/haarcascade_eye.xml')

### ULTRASONIC SENSOR SET UP ###
PIN_TRIGGER = 4
PIN_ECHO = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_TRIGGER, GPIO.OUT)
GPIO.setup(PIN_ECHO, GPIO.IN)

### DC MOTOR SET UP ###
# GPIO pin numbers 
in1 = 24
in2 = 27
in3 = 16
in4 = 22
en = 25
temp1=1
#set duty cycle
dutycycle = 40     
#motor1
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
#motor2
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
#en output
GPIO.setup(en,GPIO.OUT)
#motor 1 output
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
#motor 2 output
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
#set PWM
p=GPIO.PWM(en,1000)

### LIMIT SWITCH SET UP ###
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP) # limit switch to GPIO23

### INITIALIZE VARIABLES ###

# global variables
bg = None
# camera #
prev_x = 1      # stores 1 x value in order to compare to current x
center_x = 320  # center of the screen, aka the target value
kp = 0.1        # kp value of 0.1- low so that the motor corrects in small increments
error = 0       # initialize error as 0
# servo motors #
minAngle = 5   # minimum servo angle for both base motor and trap door motor
maxAngle = 50  # maximum servo angle for both base motor and trap door motor
keyAngles = []
foundFaces = []
# ultrasonic sensor #
targetDistance = 60
largeDistances = []

### GETTING EVERYTHING IN PLACE/INTIAL POSITIONS ###
# starting base motor servo at starting angle and the trap door closed
baseMotor.angle = 0  # base motor should start at angle 0 (alignment of cad model)
trapDoor.angle = 0   # trapdoor is closed 
time.sleep(1)

#---------------------------------------------
# FUNCTIONS
#---------------------------------------------

##########  SWEEP FUNCTION ###########
'''The sweep() function takes in 2 angle inputs and 
an integer increment input. The motor then does a sweep
between the two angles using that increment, and checks 
for significant ultrasonic distance values. Those values 
and the corresponding motor angles are then stored in 
their respective arrays.'''

def sweep(startAngle, endAngle, increment):
    print("about to sweep!")
    for x in range(startAngle, endAngle, increment):
        baseMotor.angle = x
        time.sleep(1)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)
        time.sleep(0.2)
        GPIO.output(PIN_TRIGGER, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        while GPIO.input(PIN_ECHO)==0:
                pulse_start_time = time.time()
        while GPIO.input(PIN_ECHO)==1:
                pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)
        print("Distance:",distance,"cm")
        if distance > targetDistance and distance <maxPlayingDistance and baseMotor.angle > 0:
            largeDistances.append(distance)
            keyAngles.append(baseMotor.angle)
    print("swept!")


####### RUN ARG ########
'''The run_arg function finds the running average 
over the background.'''

def run_avg(image, accumWeight):
    global bg
    # initialize the background
    if bg is None:
        bg = image.copy().astype("float")
        return

    # compute weighted average, accumulate it and update the background
    cv2.accumulateWeighted(image, bg, accumWeight)

####### SEGMENT ########
'''The segment function segments the region of the hand in the image.'''

def segment(image, threshold=25):
    global bg
    # find the absolute difference between background and current frame
    diff = cv2.absdiff(bg.astype("uint8"), image)

    # threshold the diff image so that we get the foreground
    thresholded = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)[1]

    # get the contours in the thresholded image
    (cnts, _) = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # return None, if no contours detected
    if len(cnts) == 0:
        return
    else:
        # based on contour area, get the maximum contour which is the hand
        segmented = max(cnts, key=cv2.contourArea)
        return (thresholded, segmented)

######## COUNT ########
''' The count function counts the number of fingers in the 
segmented region. '''
def count(thresholded, segmented):
    # find the convex hull of the segmented hand region
    chull = cv2.convexHull(segmented)

    # find the most extreme points in the convex hull
    extreme_top    = tuple(chull[chull[:, :, 1].argmin()][0])
    extreme_bottom = tuple(chull[chull[:, :, 1].argmax()][0])
    extreme_left   = tuple(chull[chull[:, :, 0].argmin()][0])
    extreme_right  = tuple(chull[chull[:, :, 0].argmax()][0])

    # find the center of the palm
    cX = int((extreme_left[0] + extreme_right[0]) / 2)
    cY = int((extreme_top[1] + extreme_bottom[1]) / 2)

    # find the maximum euclidean distance between the center of the palm
    # and the most extreme points of the convex hull
    distance = pairwise.euclidean_distances([(cX, cY)], Y=[extreme_left, extreme_right, extreme_top, extreme_bottom])[0]
    maximum_distance = distance[distance.argmax()]

    # calculate the radius of the circle with 80% of the max euclidean distance obtained
    radius = int(0.8 * maximum_distance)

    # find the circumference of the circle
    circumference = (2 * np.pi * radius)

    # take out the circular region of interest which has 
    # the palm and the fingers
    circular_roi = np.zeros(thresholded.shape[:2], dtype="uint8")
	
    # draw the circular ROI
    cv2.circle(circular_roi, (cX, cY), radius, 255, 1)

    # take bit-wise AND between thresholded hand using the circular ROI as the mask
    # which gives the cuts obtained using mask on the thresholded hand image
    circular_roi = cv2.bitwise_and(thresholded, thresholded, mask=circular_roi)

    # compute the contours in the circular ROI
    (cnts, _) = cv2.findContours(circular_roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # initalize the finger count
    count = 0

    # loop through the contours found
    for c in cnts:
        # compute the bounding box of the contour
        (x, y, w, h) = cv2.boundingRect(c)

        # increment the count of fingers only if -
        # 1. The contour region is not the wrist (bottom area)
        # 2. The number of points along the contour does not exceed
        #     25% of the circumference of the circular ROI
        if ((cY + (cY * 0.25)) > (y + h)) and ((circumference * 0.25) > c.shape[0]):
            count += 1

    return count

######## FACE FOLLOW  ########
''' The faceFollow() function takes frames from the current video feed, 
and looks for faces using Haar Cascade. This continues until a face is found in 
the center of the frame.'''

def faceFollow():
    direction = 1
    # starting video feed #
    
    prev_x = 1      # stores 1 x value in order to compare to current x 
    center_x = 320  # center of the screen, aka the target value
    kp = 0.1        # kp value of 0.1- low so that the motor corrects in small increments
    error = 0       # initialize error as 0
    x_val = 0

    while True:
        # grab the current frame
        img = vs.read()
        img = imutils.rotate(img, angle=180)
        img = cv2.flip(img, 1, -1) 

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(20,20)
        )  

        ## Retrieve parameters from Haar Cascade 
        for (x,y,w,h) in faces:
            #print(faces)
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]

            eyes = eyeCascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        
            x_val = int(x + (w/2))
            # Calculate x-value of person's face- center of face box 
            x_val = int(x + (w/2))
            # Calculate difference between current and previous x values
            diff = abs(x_val - prev_x)
            
            if diff > 8:
                error = center_x - x_val        # calculate error based on target value
                delta = 1*(math.floor(kp*error))  # degrees motor needs to correct
                #print('delta ', delta)
                currentAngle = baseMotor.angle
                pos = currentAngle + delta

                if pos >= minAngle and pos <= maxAngle:
                    baseMotor.angle = pos
                    time.sleep(0.1)
                    #print('pos ',pos)
                prev_x = x_val # Set current value as new previous value 

        # show the frame to our screen
        cv2.imshow("Frame", img)    
        
        key = cv2.waitKey(1) & 0xFF
        
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
            # otherwise, release the camera

        if(x_val <= (center_x+20) and x_val >= (center_x-20)):
            break
            print('face centered!')

        if len(faces)!= 1 and baseMotor.angle <= maxAngle-5:
            baseMotor.angle += 5
            time.sleep(0.1)

        if len(faces)!= 1 and baseMotor.angle > maxAngle-5:
            baseMotor.angle = 150
            time.sleep(0.1)
            baseMotor.angle = 100
            time.sleep(0.1)
            baseMotor.angle = 50
            time.sleep(0.1)
            baseMotor.angle = 5

######## RUN MOTORS  ########
''' The runMotors() function runs the flywheel 
motors to shoot the ball for a specified time 
input, as well as opens and closes the trap door.'''
   
#Run both Motors to launch ball
def runMotors(seconds):

    p.start(dutycycle)
    print("backward")

    #go forward motor 1
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    #go forward motor 2
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    time.sleep(seconds)

    trapDoor.angle =180 # opening trapdoor
    time.sleep(1)
    print("stop")

    #stop motor 1
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)

    #stop motor 2
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
    trapDoor.angle = 0  # closing trap door 

######## CHECK BALL RETURN  ########
''' The checkBallReturn() function checks the limit switch 
state and returns when the limit switch is pressed, meaning 
the ball has been returned to the robot. '''

def checkBallReturn():
    button_state = False
    while button_state == False:
        try:
            while True:
                button_state = GPIO.input(23)
                if button_state == False:
                    print('Button not pressed...')
                    time.sleep(0.5)
                else:
                    print('Button Pressed')
                    time.sleep(0.5)
                break
        except:
            GPIO.cleanup()
    print("BALL IS BACKKK")
    return True

######## LAUNCH BALL  ########
''' The launchBall() function takes a motor angle and checks if 
there is a face at that angle, and if there is a face, waits until 
it is in the center of the frame. Then, it checks for the number of 
fingers and when that requirement is satisfied, it launches the ball.'''

def launchBall(motorAngle):
    print("moving to the first face!")
    baseMotor.angle = keyAngles[motorAngle]
    print("face at this angle: ", baseMotor.angle)
    time.sleep(0.5)

    print('following face!')

    if __name__ == "__main__":
        # initialize accumulated weight
        fingers = 0
        faceFollow()
        accumWeight = 0.5
        # get the reference to the webcam    
        # region of interest (ROI) coordinates
        top, right, bottom, left = 150, 200, 325, 375
        # initialize num of frames
        num_frames = 0
        # calibration indicator
        calibrated = False
        print("checking for fingers")
        # keep looping, until interrupted

        while(True):
            # get the current frame
            img = vs.read()
            img = imutils.rotate(img, angle=180)
            img = cv2.flip(img, 1, -1) 
            # resize the frame
            frame = imutils.resize(img, width=700)
            # clone the frame
            clone = img.copy()
            # get the height and width of the frame
            (height, width) = img.shape[:2]
            # get the ROI
            roi = img[top:bottom, right:left]
            # convert the roi to grayscale and blur it
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (7, 7), 0)
            # to get the background, keep looking till a threshold is reached
            # so that our weighted average model gets calibrated

            if num_frames < 30:
                run_avg(gray, accumWeight)
                if num_frames == 1:
                    print("[STATUS] please wait! calibrating...")
                elif num_frames == 29:
                    print("[STATUS] calibration successfull...")
            else:
                # segment the hand region
                hand = segment(gray)
                # check whether hand region is segmented

                if hand is not None:
                    # if yes, unpack the thresholded image and
                    # segmented region
                    (thresholded, segmented) = hand
                    # draw the segmented region and display the frame
                    # count the number of fingers
                    fingers = count(thresholded, segmented)
                    print("fingers", fingers)

            # draw the segmented hand
            cv2.rectangle(clone, (left, top), (right, bottom), (0,255,0), 2)
            # increment the number of frames
            num_frames += 1
            # display the frame with segmented hand
            cv2.imshow("Video Feed", clone)
            # observe the keypress by the user
            keypress = cv2.waitKey(1) & 0xFF
            
            # if the user pressed "q", then stop looping
            if keypress == ord("q"):
                break
            
            if fingers > 6:
                print("FOUND ENOUGH FINGERS TO LAUNCH")
                break
    
    cv2.destroyAllWindows()
    runMotors(3)
    print('launched!')
    time.sleep(0.5)
    isball = False
    while not isball:
        isball = checkBallReturn()
        time.sleep(.5)
    return True
    
#---------------------------------------------
# MAIN LOOP
#---------------------------------------------

print("STARTING....ABOUT TO SWEEP")
# sweeps and checks for significant ultrasonic values
sweep(minAngle, maxAngle, 16) 
time.sleep(1)
# starts video stream
vs = VideoStream(src=0).start()

# loops through angles corresponding to significant ultrasonic values
for y in range(0, len(keyAngles)):
    time.sleep(1)
    # checks face, then fingers, then launches and continues!!!
    if launchBall(y) == True:
        print("moving on to the next person!!!")
        time.sleep(1) 
