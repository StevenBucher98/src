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
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(OUTPUT_PIN, GPIO.OUT, initial=GPIO.LOW)

    home_result = call_home_gantry()
    print("Homing: ", home_result.success)
    # call_move_gantry(x=500, y=0, z=0,f=250)
    # call_move_gantry(x=500, y=100, z=0,f=250)
    # call_move_gantry(x=300, y=100, z=0,f=250)
    # call_move_gantry(x=300, y=300, z=0,f=250)
    # call_move_gantry(x=300, y=600, z=0,f=250)
    # call_move_gantry(x=600, y=600, z=0,f=250)
    # call_move_gantry(x=0, y=0, z=0,f=250)
    # return
    # TODO CHECK HOMING WAS SUCCESSFUL
    if home_result.success is False:
        print("Error Homing the device")
        return 0
    
    print("Getting flower Coords:")
    coords = call_get_flower_coords()
    print("Flower Coords: ")
    print()
    c = coords[4]
    print("First Flower: ", c)

    print("Moving...")
    move_result = call_move_gantry(x=c.y, y=0, z=0, f=250)
    time.sleep(1)
    move_result = call_move_gantry(x=c.y, y=c.x, z=11, f=250)
    time.sleep(6)
    print("At flower, Clipping...")
    GPIO.output(OUTPUT_PIN, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(OUTPUT_PIN, GPIO.LOW)
    time.sleep(2)
    print("Backing off...")
    move_result = call_move_gantry(x=c.y, y=c.x - 100, z=0, f=250)
    time.sleep(1)
    c1 = coords[0]
    print("Second Flower: ", c1)
    print("Moving...")
    
    move_result = call_move_gantry(x=c1.y-15, y=c.x - 100, z=0, f=250)
    time.sleep(1)
    move_result = call_move_gantry(x=c1.y-15, y=c1.x, z=4.5, f=250)
    time.sleep(8)
    print("At flower, Clipping...")

    GPIO.output(OUTPUT_PIN, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(OUTPUT_PIN, GPIO.LOW)
    time.sleep(2)
    move_result = call_move_gantry(x=c1.y-100, y=c1.x-100, z=0, f=250)
    time.sleep(1)
    move_result = call_move_gantry(x=0, y=0, z=0, f=250)

    # print("Finished")
    # cmd = input("Ready to use toolhead? Y/N")
    # if cmd == 'Y' or cmd == 'y':
    #     GPIO.output(OUTPUT_PIN, GPIO.HIGH)
    #     time.sleep(5)
    #     GPIO.output(OUTPUT_PIN, GPIO.LOW)
    

    
        


    # command =''
    # while command != 'quit':
    #     print_coords(coords)
    #     command = input("Choose the index of the coordinate you want to traverse to ('quit' to exit): ")
    #     coord_choice = -1
    #     if command == 'quit':
    #         return
    #     try:
    #         coord_choice = int(command)
    #     except ValueError:
    #         print("Please choose an integer value")

    #     if coord_choice > len(coords) or coord_choice == -1:
    #         print("Please choose index from printed list")
    #         continue

    #     c = coords[coord_choice]
    #     print("chosen coord: ", c)

    #     print("moving...")
    #     move_result = call_move_gantry(x=c.y, y=0, z=0, f=250)
    #     time.sleep(1)
    #     move_result = call_move_gantry(x=c.y, y=c.x, z=11, f=250)
    #     time.sleep(1)
    #     print("Finished")
    #     cmd = input("Ready to use toolhead? Y/N")
    #     if cmd == 'Y' or cmd == 'y':
    #     	GPIO.output(OUTPUT_PIN, GPIO.HIGH)
    #     	time.sleep(5)
    #     	GPIO.output(OUTPUT_PIN, GPIO.LOW)
    #     else:
    #     	continue
        	

if __name__ == "__main__":
    main()
