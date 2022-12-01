import pyikfast
import numpy as np
import sys
import time

from scipy.spatial.transform import Rotation as R

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
params = {'speed': 50, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 1000, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}


params['angle_speed'] = 170
# params['angle_speed'] = 180
params['angle_acc'] = 1145


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
    # r = R.from_euler('xyz', [
    #     [angles[0], 0, 0],
    #     [0, angles[1], 0],
    #     [0, 0, angles[2]]], degrees=True)

    r = R.from_euler('zyx', xarmRPY, degrees=True)

    return list(r.as_matrix().flatten())

def toRPY(rotMat):
    rotMat3x3 = np.reshape(rotMat, (3,3))

    r = R.from_matrix(rotMat3x3)
    
    return list(r.as_euler('zxy', degrees=True))


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

        # for j, angle in enumerate(pose):
        #     for offset in addAngle:
        #         testAng = angle + offset
        #         if abs(testAng - currpose[j]) < abs(testSol[j] - currpose[j]) and abs(testAng) <= TWO_PI:
        #             testSol[j] = testAng

        # testValid = True
        
        # for angle in testSol:
        #     if testSol == 9999:
        #         testValid = False

        # if testValid:
            # validSols.append(testSol)

    # from URIKFast.cpp: 
    #
    # vector<double> sumsValid;
    # sumsValid.assign(valid_sols.size(), 0);
    # for(int i = 0; i < valid_sols.size(); i++){
    #     for(int j = 0; j < valid_sols[i].size(); j++){
    #         sumsValid[i] = pow(weight[j]*(valid_sols[i][j] - currentQ[j]), 2);
    #     }
    # }

    # print("{} valid solutions.".format(len(validSols)))

    # Does not help
    best = 0
    bestdist = sys.float_info.max
    for i, pose in enumerate(validSols):
        thisdist = np.linalg.norm(np.array(currpose)-np.array(pose))
        # print(thisdist)
        if thisdist < bestdist:
            bestdist = thisdist
            best = i

    # print("best solution {}, dist {}".format(best, bestdist))

    # return first solution
    return validSols[best]

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


# for p in range(40, 110, 10):
p=90
x = 0.400


# grid
# for z in np.arange(0.250, 0.500, 0.05):
#     for y in np.arange(-0.300, 0.400, 0.1):
# for z in np.arange(0.300, 0.701, 0.05):
#for z in np.arange(0.300, 0.701, 0.05):
#    for y in np.arange(-0.300, 0.301, 0.1):

# corners
for z in np.arange(0.250, 0.651, 0.4):
    for y in np.arange(-0.300, 0.400, 0.6):

        # move to a different spot
        # translate = [0.300, 0, 0.400]
        translate = [x, y, z]
        # rotate = toIK([0, 90, 180])
        rotate = toIK([0, p, 180])

        # do Inverse Kinematics
        results = pyikfast.inverse(translate, rotate)

        # print possible solutions
        # for result in results: 
        #     theseangles = list(result)
        #     print("-> solution (joints): ", theseangles, " ", pyikfast.forward(theseangles))

        newPose = selectSolution(results, startPose)
        print("new pose IK (radians):\n{}".format(newPose))

        translate, rotate  = pyikfast.forward(newPose)
        print("new position FK (translate, rotate):\n{}\n{}".format(translate, rotate))
        print("new position: "
            +",".join([" {0:.4f}".format(el) for el in translate])
            +",".join([" {0:.4f}".format(el) for el in toRPY(rotate)])
            +"\n"
            )
        
        newPose = list(np.degrees(newPose))
        # move to result
        arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        # arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=False, radius=-1.0)
        # print("new pose IK (degrees):\n{}\n".format(newPose))

arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
