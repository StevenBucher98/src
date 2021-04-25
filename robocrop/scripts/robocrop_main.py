#!/usr/bin/env python
import sys
import rospy

from robocrop.srv import *

def main():
    #rospy.wait_for_service('add_two_ints')
    print("robocrop main sequence")
    rospy.wait_for_service('get_flower_coords')
    try:
        get_flower_coords = rospy.ServiceProxy('get_flower_coords', AddTwoInts)
        coords = get_flower_coords()
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()