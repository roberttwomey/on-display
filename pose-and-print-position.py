#!/usr/local/bin/python3

# Simple text programs for python control of xARM
# this one reads in a series of pen recorder coordinates and 
# replays them on the xarm

# reference for https://github.com/xArm-Developer/xArm-Python-SDK

from xarm.wrapper import XArmAPI
import time
import sys
import numpy as np
import argparse
#import cv2

def main():
 
    xarm_rest_pos = [220, 0, 120.5, 180, 0, 0]
    xarm_speed = 350
    xarm_mvacc = 800
    xarm_height = 15.0 #mm
 
    # connect to drawing machine
    print("Initializing xArm...")
    arm = XArmAPI('192.168.4.15')
    # arm.connect()

    # from http://download.ufactory.cc/xArm_Python_SDK/1001-xArm-linear%20motion-example1.py
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    # arm.set_tcp_jerk(1000)
    # arm.set_joint_jerk(20,is_radian=True)
    # arm.set_pause_time(1)
    # arm.save_conf()

    arm.set_position(*xarm_rest_pos, speed=350, wait=True)
    print("moved to rest_position")
    sys.stdout.flush()

    arm.set_mode(2) # joint teaching
    arm.set_state(0)

    try:
        
        while True:
            print('* position:', arm.position)
            # print('\t angles:', arm.angles)
            time.sleep(0.1)
            sys.stdout.flush()
    except (KeyboardInterrupt):
        print("exiting...")

        pass

    # close arm
    arm.set_mode(0)
    arm.set_state(state=0)

    # move to rest
    # arm.set_position(*xarm_rest_pos, wait=True)
    arm.reset(wait=True)
    arm.disconnect()
    print("\n")

if __name__ == "__main__":
    main()
