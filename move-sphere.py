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

"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""
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


pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.4.15')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

variables = {}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}


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

# Rotation
if not params['quit']:
    params['angle_acc'] = 1145
if not params['quit']:
    params['angle_speed'] = 80
    # if params['quit']:
    
    if arm.error_code == 0 and not params['quit']:
        # code = arm.set_servo_angle(angle=[0.1, -34.9, -0.1, 1.6, 0, -63.5, 0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        code = arm.set_servo_angle(angle=frontForwardAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)

        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))


print('* position:', arm.position)
# radius = np.linalg.norm(arm.position[:3])
# print('\tradius:', radius)

z_offset = 400.0

x = arm.position[0]
y = arm.position[1]
z = arm.position[2] - z_offset
roll = arm.position[3]
pitch = arm.position[4]
starting_pitch = pitch
yaw = arm.position[5]

rotation = math.atan(y/x) # opposite/adjacent
elevation = math.atan(z/x) # opposite/adjacent
print(math.degrees(rotation), math.degrees(elevation))

radius = x

newx = radius*math.cos(rotation)
newy = radius*math.sin(rotation)
newz = radius*math.sin(elevation)+z_offset

print(newx, newy, newz)

# pan in y direction

# for i in range(5):
#     elevation += math.radians(10)

#     newx = radius*math.cos(rotation)
#     newy = radius*math.sin(rotation)
#     newz = radius*math.sin(elevation)+z_offset
#     newx *= math.cos(elevation)
#     pitch = (starting_pitch+math.degrees(elevation))

#     print(newx, newy, newz, roll, pitch, yaw)
#     arm.set_position(newx, newy, newz, roll, pitch, yaw, radius=0, speed=200, mvacc=2000, wait=False)#, speed=500, relative=False, wait=False)

# for i in range(5):
#     elevation += math.radians(-10)

#     newx = radius*math.cos(rotation)
#     newy = radius*math.sin(rotation)
#     newz = radius*math.sin(elevation)+z_offset
#     newx *= math.cos(elevation)
#     pitch = (starting_pitch+math.degrees(elevation))

#     print(newx, newy, newz, roll, pitch, yaw)
#     arm.set_position(newx, newy, newz, roll, pitch, yaw, radius=0, speed=200, mvacc=2000, wait=False)#, speed=500, relative=False, wait=False)


# pan in x plane

for i in range(5):
    rotation += math.radians(10)

    newx = radius*math.cos(rotation)
    newy = radius*math.sin(rotation)
    newz = radius*math.sin(elevation)+z_offset
    yaw = ((180+math.degrees(rotation))%180)-180
    print(newx, newy, newz, roll, pitch, yaw)

    arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)


for i in range(10):
    rotation += math.radians(-10)

    newx = radius*math.cos(rotation)
    newy = radius*math.sin(rotation)
    newz = radius*math.sin(elevation)+z_offset
    yaw = (180+math.degrees(rotation))
    if yaw < -180:
        yaw += 360
    if yaw > 180: 
        yaw -=360

    print(newx, newy, newz, roll, pitch, yaw)

    arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)

for i in range(5):
    rotation += math.radians(10)

    newx = radius*math.cos(rotation)
    newy = radius*math.sin(rotation)
    newz = radius*math.sin(elevation)+z_offset
    yaw = (180+math.degrees(rotation))
    if yaw < -180:
        yaw += 360
    if yaw > 180: 
        yaw -=360

    print(newx, newy, newz, roll, pitch, yaw)

    arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)


# for i in range(5):
#     rotation += math.radians(10)

#     newx = radius*math.cos(rotation)
#     newy = radius*math.sin(rotation)
#     newz = radius*math.sin(elevation)+z_offset
#     yaw = ((180+math.degrees(rotation))%180)-180
#     print(newx, newy, newz, roll, pitch, yaw)

#     arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=200, mvacc=2000, wait=False, radius=0)#, speed=500, relative=False, wait=False)


# for i in range(5):
#     rotation += math.radians(-10)

#     newx = radius*math.cos(rotation)
#     newy = radius*math.sin(rotation)
#     newz = radius*math.sin(elevation)+z_offset

#     print(newx, newy, newz, roll, pitch, yaw)

#     arm.set_position(newx, newy, newz, roll, pitch, yaw, speed=1000, mvacc=2000, wait=True)#, speed=500, relative=False, wait=False)


# look forward but retracted
# code = arm.set_servo_angle(angle=frontBackAngle, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
# print('* position:', arm.position)
# print('\tradius:', np.linalg.norm(arm.position[:3]))




# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
