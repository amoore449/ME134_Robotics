import time, math
#from adafruit_servokit import ServoKit

#kit = ServoKit(channels=16)

def IK_Solve(x, y, z):
    #Convert to polar coordinates
    x = x/6
    y = y/6
    theta = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2) - 9

    #Base angle is equal to theta
    theta1 = theta + math.pi/4

    #Inverse Kinematic Equations for 3-link planar arm
    length1 = 102.7
    length2 = 96.7
    length3 = 35.1

    #Set end effector coordinates
    x_e = r
    y_e = z
    phi_e = math.pi/2

    #Find wrist coordinates
    x_w = x_e - length3 * math.cos(phi_e)
    y_w = y_e - length3 * math.sin(phi_e)
    
    theta3 = math.pi - math.acos((length1**2 + length2**2 - x_w**2 - y_w**2)/(2*length1*length2))    
    theta3_out = (theta3)

    alpha = math.atan2(y_w, x_w)

    theta2 = alpha - math.acos((x_w**2 + y_w**2 + length1**2 - length2**2)/(2*length1*math.sqrt(x_w**2 + y_w**2)))
    theta2_out = 3*math.pi/2 + (theta2)
    theta4 = phi_e - theta2 - theta3
    theta4_out = (theta4)

    
    theta1_out = catch_all(theta1) 
    theta2_out = catch_all(theta2_out) 
    theta3_out = catch_all(theta3_out) 
    theta4_out = catch_all(theta4_out) 
    print(theta2_out)
    print(theta3_out)
    print(theta4_out)

    return [theta1_out, theta2_out, theta3_out, theta4_out]

def catch_all(theta):
    rad2deg = 180/math.pi
    theta_deg = rad2deg*theta

    if theta_deg > 180:
        theta_deg = 180
    elif theta_deg < 0:
        theta_deg = 0
    return theta_deg

# kit.servo[0].angle = 180
# kit.continuous_servo[1].throttle = 1
# time.sleep(1)
# kit.continuous_servo[1].throttle = -1
# time.sleep(1)
# kit.servo[0].angle = 0
# kit.continuous_servo[1].throttle = 0
