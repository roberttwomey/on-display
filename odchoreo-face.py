#!/usr/bin/env python3
import sys
import math
import time
import datetime
import random
import traceback
import threading
import numpy as np


# -------- cv setup --------
import cv2

showDebug = True
closeSizeCutoff = 100 #275.0
maxCutoff = 400

# To capture video from webcam. 
cap = cv2.VideoCapture(0)

capWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
capHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Load the cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# load text
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
thickness              = 1
lineType               = 2

tiltAng = 0
panAng = 5
timeLastSeen = time.time()
updateInterval = 0.1
tLastUpdated = time.time()

# ---- IK fast stuff
import pyikfast
from scipy.spatial.transform import Rotation as R

TWO_PI = np.pi * 2.0

xarmJointLimits = [
    [-6.283185307179586, 6.283185307179586],
    [-2.059, 2.0944],
    [-6.283185307179586, 6.283185307179586],
    [-0.19198, 3.927],
    [-6.283185307179586, 6.283185307179586],
    [-1.69297, 3.141592653589793],
    [-6.283185307179586, 6.283185307179586]
]

def toIK(xarmRPY):
    # r = R.from_euler('zyx', xarmRPY, degrees=True)
    r = R.from_euler('xyz', xarmRPY, degrees=True)

    return list(r.as_matrix().flatten())

def toRPY(rotMat):
    rotMat3x3 = np.reshape(rotMat, (3,3))

    r = R.from_matrix(rotMat3x3)
    
    return list(r.as_euler('zyx', degrees=True))


def selectSolution(solutions, currpose):
    selectedSolution = 0
    valid = [True] * len(solutions)
    
    # reject solutions outside of joint limits
    for i, pose in enumerate(solutions):
        for j, angle in enumerate(pose):
            if angle < xarmJointLimits[j][0] or angle > xarmJointLimits[j][1]:
                # print("angle {} in solution {} is out of range: {}".format(j, i, angle))
                valid[i]=False
                break    

    # testSol = [9999]*7 # 7 DOF
    # addAngle = [-1*TWO_PI, 0, TWO_PI]

    validSols = []

    for i, pose in enumerate(solutions):
        if valid[i] == False:
            continue
        else: 
            validSols.append(pose)

    # print("{} valid solutions.".format(len(validSols)))

    # Does not help
    best = None
    bestdist = sys.float_info.max
    for i, pose in enumerate(validSols):
        thisdist = np.linalg.norm(np.array(currpose)-np.array(pose))
        # print(thisdist)
        if thisdist < bestdist:
            bestdist = thisdist
            best = i

    # print("best solution {}, dist {}".format(best, bestdist))
    if best is not None: 
        return validSols[best]
    else:
        return None

# -------- begin xArm stuff --------


try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

frontForwardAngle = [0, 2.5, 0, 37.3, 0, -57.3, 0]
frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]
stretchout = [-350.0, 0, 0, 180, -350, 45, 45]

pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.4.15')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

variables = {}
params = {
    'speed': 100, 'acc': 2000, 
    'angle_speed': 10, #170, 
    'angle_acc': 1145, 
    'events': {}, 'variables': variables, 
    'callback_in_thread': True, 'quit': False
    }

slowspeed = 80


# Register error/warn changed callback
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(connect_changed_callback)


# move to start point
# frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]
# arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
# startPose = list(np.radians(frontBackAngle))

arm.set_servo_angle(angle=stretchout, speed=80, mvacc=params['angle_acc'], wait=True, radius=-1.0)
startPose = list(np.radians(stretchout))
print("current position (xArm): [{:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}]".format(*arm.position))

startAngles = arm.angles
# convert to radians
startPose = list(np.radians(startAngles))   

# print('start pose (angles): ', arm.angles)
# print("start pose (radians): {}".format(startPose))

# calculate translation and rotation
translate, rotate  = pyikfast.forward(startPose)
# print("start position FK (translate, rotate): \n{}\n{}".format(translate, rotate))
# print("start position (API): {}".format(arm.position))
# print("matrix form: {}".format(toIK(arm.position[3:6])))

z_offset = 400.0#400.0

starting_position = arm.position

x = 400.0 #arm.position[0]
y = arm.position[1]
z = arm.position[2] - z_offset
roll = arm.position[3] # degrees
pitch = arm.position[4] # degrees
starting_pitch = pitch
yaw = arm.position[5] # degrees

rotation = math.degrees(math.atan(y/x)) # opposite/adjacent
elevation = math.degrees(math.atan(z/x)) # opposite/adjacent
# print("rotation and elevation: ", rotation, elevation)
# print("rpy: ", [roll, pitch, yaw])

start_radius = x

timeLastMoved = time.time()-10
moveCadence = 10.0
faceScore = 0

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

while True:
    try:
        try: 


            now = time.time()

            # Read the frame
            _, img = cap.read()
            
            if img is not None:
                # Convert to grayscale
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                # Detect the faces
                faces = face_cascade.detectMultiScale(gray, 1.1, 4, minSize=(150,150), maxSize=(600, 600))
                # Draw the rectangle around each face
                
                facecount = 0
                maxSize = 0
                maxIndex = -1
                maxLoc = None

                for (x, y, w, h) in faces:

                    if showDebug:
                        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                        cv2.putText(img,"{} x {}".format(w,h), (x,y), 
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)
                    
                    # check how close the face is            
                    if w > maxSize and w < maxCutoff: 
                        maxSize = w
                        maxIndex = facecount
                        maxLoc = (x, y, w, h)
                        facecount += 1
                
                if facecount > 0:
                    faceScore += 1

            timeElapsed = time.time() - timeLastMoved
            cv2.putText(img,"{:.2f} {}".format(timeElapsed, faceScore), (100, 100), 
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)

            faceScore-=1
            if faceScore < 0:
                faceScore = 0

            if arm.get_is_moving():
                timeLastMoved = time.time()

            elif time.time() - timeLastMoved > moveCadence: 
            # and faceScore == 0:

                print("current position (xArm): [{:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}]".format(*arm.position))

                # rotation = random.uniform(-110, 110)
                rotation = random.uniform(-145, -145)
                elevation = random.uniform(-30, 60)
                extension = random.uniform(-50, 300)
                deltapitch = random.uniform(-90, 90)

                radius = start_radius+extension
                pitch = starting_pitch+elevation+deltapitch
                # pitch = constrain(pitch, -45, 45)

                # calc target position move on surface of sphere
                newx = radius*math.cos(math.radians(rotation))
                newy = radius*math.sin(math.radians(rotation))
                newz = radius*math.sin(math.radians(elevation))+z_offset
                
                roll = 0
                yaw = rotation - 180
                if yaw < -180:
                    yaw += 360
                elif yaw > 180:
                    yaw -= 360

                rotMat = toIK([roll, pitch, yaw])

                # print("target rotation, elevation, radius: {:0.4f}, {:0.4f}, {:0.4f}".format(rotation, elevation, radius))
                print("target: [{:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}, {:0.4f}]".format(newx, newy, newz, roll, pitch, yaw))

                translate = [coord / 1000.0 for coord in [newx, newy, newz]]
                results = pyikfast.inverse(translate, rotMat)
                currPose = list(np.radians(arm.angles))
                newPose = selectSolution(results, currPose)

                if newPose is not None:
                    # print("new pose IK (radians):\n{}".format(newPose))
                    newPose = list(np.degrees(newPose))

                    # move to result
                    arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=False, radius=-1.0)

                    # time.sleep(3)
                    timeLastMoved = time.time()
                else:
                    print(" -- xxx -- unachievable")


            # Display video
            cv2.imshow('img', img)

            # Stop if escape key is pressed
            k = cv2.waitKey(30) & 0xff
            if k==27:
                break

        except (KeyboardInterrupt):
            arm.set_state(state=3) # pause
            print("paused")
            input("Press enter to continue, Ctrl-C to quit")
            arm.set_state(state=0)
            continue
    
    except (KeyboardInterrupt):
        # arm.set_state(state=0)
        print("exiting...")
        break

print("Done...")


arm.set_servo_angle(angle=frontForwardAngle, speed=slowspeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)
# arm.set_servo_angle(angle=stretchout, speed=slowspeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)

# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
