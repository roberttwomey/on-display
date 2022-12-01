#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
"""
import sys
import math
import time
import datetime
import random
import traceback
import threading
import numpy as np


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
    'angle_speed': 170, #170, 
    'angle_acc': 1145, 
    'events': {}, 'variables': variables, 
    'callback_in_thread': True, 'quit': False
    }


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
frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]
arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
startPose = list(np.radians(frontBackAngle))

startAngles = arm.angles
# convert to radians
startPose = list(np.radians(startAngles))   

# print('start pose (angles): ', arm.angles)
# print("start pose (radians): {}".format(startPose))

# calculate translation and rotation
translate, rotate  = pyikfast.forward(startPose)
print("start position FK (translate, rotate): \n{}\n{}".format(translate, rotate))
print("start position (API): {}".format(arm.position))
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

rotation = math.atan(y/x) # opposite/adjacent
elevation = math.atan(z/x) # opposite/adjacent
print("rotation and elevation: ", math.degrees(rotation), math.degrees(elevation))
print("rpy: ", [roll, pitch, yaw])

radius = x
extension = 200.0
radius += extension

# newx = radius*math.cos(rotation)
# newy = radius*math.sin(rotation)
# newz = radius*math.sin(elevation)+z_offset
# print("new xyz: ", newx, newy, newz)

## FASTER PAN

# radius *= 0.5 # min
# radius *= 1.5 # max though locks on overhead
elevation += math.radians(20)
pitch = starting_pitch + math.degrees(elevation)

# pan in y plane
for j in range(5):
    
    # pan in x plane
    for i in range(4):
        
        print("target rotation and elevation: ", math.degrees(rotation), math.degrees(elevation))
        newx = radius*math.cos(rotation)
        newy = radius*math.sin(rotation)
        newz = radius*math.sin(elevation)+z_offset
        # yaw = ((180+math.degrees(rotation))%180)-180
        yaw = math.degrees(rotation)-180
        # yaw = math.degrees(rotation)90
        # if yaw < -180:
        #     yaw += 360
        # if yaw > 180: 
        #     yaw -=360

        print("current position (xArm): {}".format(arm.position))
        print("target position: {}".format([newx, newy, newz, roll, pitch, yaw]))

        translate = [coord / 1000.0 for coord in [newx, newy, newz]]
        rotate = toIK([roll, pitch, yaw])
        # print("current matrix (translate, rotate): \n{}\n{}\n".format(translate, rotate))

        # do Inverse Kinematics
        results = pyikfast.inverse(translate, rotate)
        newPose = selectSolution(results, startPose)
        # print("new pose IK (radians):\n{}".format(newPose))

        newPose = list(np.degrees(newPose))
        # move to result
        arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        # arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)

        rotation += math.radians(10)

    for i in range(8):

        newx = radius*math.cos(rotation)
        newy = radius*math.sin(rotation)
        newz = radius*math.sin(elevation)+z_offset
        # yaw = (180+math.degrees(rotation))
        yaw = math.degrees(rotation)+180
        if yaw < -180:
            yaw += 360
        if yaw > 180: 
            yaw -=360

        # print(newx, newy, newz, roll, pitch, yaw)
        # arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)
        print("current position (xArm): {}".format([newx, newy, newz, roll, pitch, yaw]))


        translate = [coord / 1000.0 for coord in [newx, newy, newz]]
        rotate = toIK([roll, pitch, yaw])

        # do Inverse Kinematics
        results = pyikfast.inverse(translate, rotate)

        newPose = selectSolution(results, startPose)
        # print("new pose IK (radians):\n{}".format(newPose))

        newPose = list(np.degrees(newPose))
        # move to result
        arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

        rotation += math.radians(-10)
    
    for i in range(4):    

        newx = radius*math.cos(rotation)
        newy = radius*math.sin(rotation)
        newz = radius*math.sin(elevation)+z_offset
        yaw = (180+math.degrees(rotation))
        if yaw < -180:
            yaw += 360
        if yaw > 180: 
            yaw -=360

        # print(newx, newy, newz, roll, pitch, yaw)
        # arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)
        print("current position (xArm): {}".format([newx, newy, newz, roll, pitch, yaw]))
        translate = [coord / 1000.0 for coord in [newx, newy, newz]]
        rotate = toIK([roll, pitch, yaw])

        # do Inverse Kinematics
        results = pyikfast.inverse(translate, rotate)

        newPose = selectSolution(results, startPose)
        # print("new pose IK (radians):\n{}".format(newPose))

        newPose = list(np.degrees(newPose))
        # move to result
        arm.set_servo_angle(angle=newPose, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

        rotation += math.radians(10)

    # location of toolhead
    elevation += math.radians(10)

    # pitch of toolhead
    pitch = starting_pitch+math.degrees(elevation)


print("Done...")

arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
