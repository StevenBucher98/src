#!/usr/bin/env python3
import pyzed.sl as sl
import numpy as np
import cv2
import sys

from camera.srv import get_flower_coords, get_flower_coordsResponse
import rospy

def handle_srv_call(req):
    return get_flower_coordsResponse([1,2,3], [4,5,6])

def get_flower_coords_server():
    while(True):
        rospy.init_node('get_flower_coords')
        s = rospy.Service('get_flower_coords', get_flower_coords, handle_srv_call)
        print("awaiting call")
        rospy.spin()

if __name__ == "__main__":
    get_flower_coords_server()
