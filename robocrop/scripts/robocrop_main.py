#!/usr/bin/env python
import sys
import rospy
import inspect
import time
import RPi.GPIO as GPIO
from robocrop.srv import *

SERVICE_TIMEOUT = 5
OUTPUT_PIN = 18

def call_get_flower_coords():
    rospy.wait_for_service('get_flower_coords', timeout=SERVICE_TIMEOUT)
    try:
		get_flower_coords_func = rospy.ServiceProxy('get_flower_coords', get_flower_coords)
		coords = get_flower_coords_func()
		#print(coords.coords)
		return coords.coords
    except rospy.ServiceException as e:
        print("get_flower_coords service call failed: %s"%e)
        

def call_move_gantry(x,y,z,f):
    rospy.wait_for_service('move_gantry', timeout=SERVICE_TIMEOUT)
    try:
		move_gantry_func = rospy.ServiceProxy('move_gantry', move_gantry)
		result = move_gantry_func(x,y,z,f)
		return result
    except rospy.ServiceException as e:
        print("move_gantry service call failed: %s"%e)

def call_home_gantry():
    rospy.wait_for_service('home_gantry', timeout=SERVICE_TIMEOUT)
    try:
		home_gantry_func = rospy.ServiceProxy('home_gantry', home_gantry)
		result = home_gantry_func()
		return result
    except rospy.ServiceException as e:
        print("home_gantry service call failed: %s"%e)

def print_coords(coords):
	for coord_idx, coord in enumerate(coords):
		print("-----------------------------")
		print("Index: ", coord_idx)
		print(coord)


def main():
    print("RoboCrop Main Sequence Begin")
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(OUTPUT_PIN, GPIO.OUT, initial=GPIO.LOW)
    home_result = call_home_gantry()
    print(type(home_result.success))
    # TODO CHECK HOMING WAS SUCCESSFUL
    if home_result.success is False:
        print("Error Homing the device")
        return 0
    
    coords = call_get_flower_coords()

    command =''
    while command != 'quit':
        print_coords(coords)
        command = input("Choose the index of the coordinate you want to traverse to ('quit' to exit): ")
        coord_choice = -1
        if command == 'quit':
            return
        try:
            coord_choice = int(command)
        except ValueError:
            print("Please choose an integer value")

        if coord_choice > len(coords) or coord_choice == -1:
            print("Please choose index from printed list")
            continue

        c = coords[coord_choice]
        print("chosen coord: ", c)

        print("moving...")
        move_result = call_move_gantry(x=c.x, y=0, z=0,f=250)
        time.sleep(3)
        move_result = call_move_gantry(x=c.x, y=c.y, z=0,f=250)
        time.sleep(5)
        print("Finished")
        cmd = input("Ready to use toolhead? Y/N")
        if cmd == 'Y' or cmd == 'y':
        	GPIO.output(OUTPUT_PIN, GPIO.HIGH)
        	time.sleep(5)
        	GPIO.output(OUTPUT_PIN, GPIO.LOW)
        else:
        	continue
        	

if __name__ == "__main__":
    main()
