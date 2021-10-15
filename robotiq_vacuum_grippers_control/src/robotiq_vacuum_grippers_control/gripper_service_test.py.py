#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotiq_vacuum_grippers_control.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('gripper_cmd')
    try:
        add_two_ints = rospy.ServiceProxy('gripper_cmd', gripper_cmd)
        resp1 = add_two_ints(1)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])        

