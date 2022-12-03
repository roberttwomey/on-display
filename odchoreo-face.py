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
import mediapipe as mp
import cv2

# mediapipe functionality
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

showDebug = True
closeSizeCutoff = 100 #275.0
maxCutoff = 400

# To capture video from webcam. 
cap = cv2.VideoCapture(0)

capWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
capHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

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

homespeed = 80


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
startAngle = frontForwardAngle
# startAngle = stretchout

arm.set_servo_angle(angle=startAngle, speed=homespeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)
startPose = list(np.radians(startAngle))
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

# model_selection: 
# 0 type model: detect faces within 2 meters
# 1 type model: detect faces within 5 meters
# min_detection_confidence: 0-1.0 0%-100%

with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.9) as face_detection:  
    while cap.isOpened():
        try:
            try: 
                now = time.time()

                # Read the frame
                success, img = cap.read()
        
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                img.flags.writeable = False
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                results = face_detection.process(img)

                # Draw the face detection annotations on the image.
                img.flags.writeable = True
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                maxSize = 0
                maxLoc = None
                xError = 0
                yError = 0
                facecount = 0

                if results.detections:
                    for detection in results.detections:
                        # print(detection.location_data.relative_bounding_box)
                        mp_drawing.draw_detection(img, detection)
                        bbox = detection.location_data.relative_bounding_box

                        thiswidth = bbox.width * capWidth
                        # check how close the face is 

                        # if thiswidth > maxSize and thiswidth < maxCutoff and bbox.xmin > 0.01 and bbox.ymin > 0.01: 
                        if thiswidth > maxSize and bbox.xmin > 0.01 and bbox.ymin > 0.01: 
                            maxSize = thiswidth
                            maxLoc = bbox
                            facecount+=1

                    # if (facecount > 0) and (maxSize > closeSizeCutoff) and (maxSize < maxCutoff):
                    # no thresholds
                    if facecount > 0:
                        # Work with largest face on frame
                        
                        # print(x, maxSize, closeSizeCutoff)
                        # calculate distance of largest face from center of image
                        x = int(maxLoc.xmin*capWidth)
                        y = int(maxLoc.ymin*capHeight)
                        w = int(maxLoc.width*capWidth)
                        h = int(maxLoc.height*capHeight)

                        xError = (0.5*capWidth-(x+w*0.5))/capWidth
                        yError = (0.5*capHeight-(y+h*0.5))/capHeight

                        # show face                    
                        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 10)

                        # show error
                        cv2.line(img, (int(capWidth/2), int(capHeight/2)), (int(x+w/2), int(capHeight/2)), (0, 0, 255), 1)
                        cv2.line(img, (int(capWidth/2), int(capHeight/2)), (int(capWidth/2), int(y+h/2)), (0, 0, 255), 1)


                        if time.time() - tLastUpdated > updateInterval:
                            
                            tLastUpdated = time.time()

                        timeLastSeen = time.time()
                        tLastUpdate = time.time()

                        faceScore+=2

                        if faceScore > 10:
                            faceScore = 10

                # Relax position back to trajectory
                # else: 

  

                    # if timeElapsed > 5.0 and timeElapsed < 5.0:
                        
                    #     # print("relaxing to front")
                    #     cv2.putText(img,"RELAXING", (10, 110), 
                    #         font, 
                    #         fontScale,
                    #         fontColor,
                    #         thickness,
                    #         lineType)

                    #     currPose = list(np.radians(arm.angles))


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

                if faceScore > 5:
                    arm.set_state(3)
                else:
                    if arm.get_state != 0:
                        arm.set_state(0)

                if time.time() - timeLastMoved > moveCadence and faceScore < 5: 
                    
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
            arm.set_state(state=4) # stop. clears moves
            arm.set_state(state=0) # sport. get going home
            print("exiting...")
            break

print("Done...")


arm.set_servo_angle(angle=frontForwardAngle, speed=homespeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)
# arm.set_servo_angle(angle=stretchout, speed=slowspeed, mvacc=params['angle_acc'], wait=True, radius=-1.0)

# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
