import pyikfast
import numpy as np
import sys
import time
import math
from scipy.spatial.transform import Rotation as R

# -------- cv setup --------
import cv2

showDebug = True
closeSizeCutoff = 150 #275.0
maxCutoff = 300

# To capture video from webcam. 
cap = cv2.VideoCapture(1)

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


# -------- xArm setup --------

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
    'angle_speed': 170, 
    'angle_acc': 1145, 
    'events': {}, 'variables': variables, 
    'callback_in_thread': True, 'quit': False
    }




# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(connect_changed_callback)


# -------- main code --------


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

# def toXarm(translate, rotate):


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

# move to start point
frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]
arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
startPose = list(np.radians(frontBackAngle))

# degrees
startAngles = arm.angles
print('start pose (angles):', arm.angles)

# convert to radians
startPose = list(np.radians(startAngles))   
print("start pose (radians): \n{}\n".format(startPose))

# calculate translation and rotation
translate, rotate  = pyikfast.forward(startPose)
print("start position FK (translate, rotate): \n{}\n{}\n".format(translate, rotate))

# print("start position (API): {}".format(arm.position))
# print("matrix form: {}".format(toIK(arm.position)))

# [223.926, -0.0, 399.922, 0.0, -89.99998127603166, 180.00001984784282]


z_offset = 400.0#400.0

starting_position = arm.position

xpos = 400.0 #arm.position[0]
ypos = arm.position[1]
zpos = arm.position[2] - z_offset
roll = arm.position[3] # degrees
pitch = arm.position[4] # degrees
starting_pitch = pitch
yaw = arm.position[5] # degrees

rotation = math.atan(ypos/xpos) # opposite/adjacent
elevation = math.atan(zpos/xpos) # opposite/adjacent
print("rotation and elevation: ", math.degrees(rotation), math.degrees(elevation))
print("rpy: ", [roll, pitch, yaw])

radius = xpos
extension = 0.0
radius += extension
# elevation += math.radians(20)
pitch = starting_pitch + math.degrees(elevation)


xError=0
yError=0

# grid
# for z in np.arange(0.250, 0.500, 0.05):
#     for y in np.arange(-0.300, 0.400, 0.1):
# for z in np.arange(0.300, 0.701, 0.05):
#for z in np.arange(0.300, 0.701, 0.05):
#    for y in np.arange(-0.300, 0.301, 0.1):

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
                
                if (facecount > 0) and (maxSize > closeSizeCutoff) and (maxSize < maxCutoff):
                    # print(x, maxSize, closeSizeCutoff)
                    # calculate distance of first face from center of image
                    (x, y, w, h) = maxLoc

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

                else: 
                    timeElapsed = time.time() - timeLastSeen
                    cv2.putText(img, "{:.2f}".format(timeElapsed), (10, 70), 
                        font, 
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)

                    if timeElapsed > 1.0 and timeElapsed < 5.0:
                        
                        # print("relaxing to front")
                        cv2.putText(img,"RELAXING", (10, 110), 
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)

                        if time.time() - tLastUpdated > updateInterval:

                            # relax to front position
                            currAngle = arm.angles
                            weight=0.9
                            # weight = 0.25
                            # destAngle = [(1.0-weight)*currAngle[i]+weight*frontAngle[i] for i in range(len(frontAngle))]
                            # arm.set_servo_angle(angle=destAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=False, radius=-1.0)

                            tLastUpdated = time.time()

            # ======== MOVEMENT ========

            print("target rotation and elevation: ", math.degrees(rotation), math.degrees(elevation))
            newx = radius*math.cos(rotation)
            newy = radius*math.sin(rotation)
            newz = radius*math.sin(elevation)+z_offset
            # yaw = ((180+math.degrees(rotation))%180)-180

            camFOV = math.radians(103) # razer kiyo pro is 103 degrees FOV wide angle
            radPerPix = camFOV/capWidth
            
            rotation += xError*math.radians(103)
            elevation += yError*math.radians(103)*0.5625

            yaw = math.degrees(rotation)-180
            pitch = starting_pitch+math.degrees(elevation)

            print("current position (xArm): {}".format(arm.position))
            print("target position: {}".format([newx, newy, newz, roll, pitch, yaw]))

            translate = [coord / 1000.0 for coord in [newx, newy, newz]]
            rotMat = toIK([roll, pitch, yaw])

            # do Inverse Kinematics
            results = pyikfast.inverse(translate, rotMat)
            currPose = list(np.radians(arm.angles))
            newPose = selectSolution(results, currPose)

            if newPose is not None:
                # print("new pose IK (radians):\n{}".format(newPose))
                newPose = list(np.degrees(newPose))

                # move to result
                arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=10.0)


                # time.sleep(3)
            else:
                print("found an unachievable position: ", [newx, newy, newz, roll, pitch, yaw])

            # Display video
            cv2.imshow('img', img)

            print("elapsed= ", time.time()-now)

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
        arm.set_state(state=0)
        print("exiting...")
        break

print("Done...")

# Release the VideoCapture object
cap.release()


arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
