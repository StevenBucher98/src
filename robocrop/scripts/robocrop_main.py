#!/usr/bin/env python
import sys
import rospy
import inspect
from robocrop.srv import *

def call_get_flower_coords():
    rospy.wait_for_service('get_flower_coords')
    try:
		get_flower_coords_func = rospy.ServiceProxy('get_flower_coords', get_flower_coords)
		coords = get_flower_coords_func()
		print(coords.coords)
		#print(type(coords))
		#print(inspect.getmembers(coords))
		#for c in coords.coords:
		#	print(c)
		#	print("_______")
		return coords.coords
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        

def call_move_gantry(x,y,z,f):
    rospy.wait_for_service('move_gantry')
    try:
		move_gantry_func = rospy.ServiceProxy('move_gantry', move_gantry)
		result = move_gantry_func(x,y,z,f)
		return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_home_gantry():
    rospy.wait_for_service('home_gantry')
    try:
		home_gantry_func = rospy.ServiceProxy('home_gantry', home_gantry)
		result = home_gantry_func()
		return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    #rospy.wait_for_service('add_two_ints')
    
    
    print("RoboCrop Main Sequence Begin")
    home_result = call_home_gantry()
    print(home_results)
    # TODO CHECK HOMING WAS SUCCESSFUL
    
    coords = call_get_flower_coords()
    for c in coords:
        print(c)
        move_result = call_move_gantry(x=c.x, y=c.y, z=0,f=100)
        #TODO CHECK MOVE IS SUCCESSFUL

if __name__ == "__main__":
    main()
