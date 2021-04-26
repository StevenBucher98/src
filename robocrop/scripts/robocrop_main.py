#!/usr/bin/env python
import sys
import rospy
import inspect
from robocrop.srv import *

def main():
    #rospy.wait_for_service('add_two_ints')
    print("robocrop main sequence")
    rospy.wait_for_service('get_flower_coords')
    try:
		get_flower_coords_func = rospy.ServiceProxy('get_flower_coords', get_flower_coords)
		coords = get_flower_coords_func()
		print(coords.coords)
		#print(type(coords))
		#print(inspect.getmembers(coords))
		for c in coords.coords:
			print(c.x)
			print(type(c))
			print("_______")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
