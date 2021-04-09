#!/usr/bin/env python3
import pyzed.sl as sl
import numpy as np
import cv2
import sys
import rospy

from camera.srv import get_flower_coords, get_flower_coordsResponse
from camera.msg import Coord

def handle_srv_call(req):
    coord1 = Coord()
    coord2 = Coord()
    coord1.x = 1
    coord1.y = 2
    coord1.z = 3
    coord2.x = 4
    coord2.y = 5
    coord2.z = 6
    return get_flower_coordsResponse([coord1, coord2])

def get_flower_coords_server():
    while(True):
        rospy.init_node('get_flower_coords')
        s = rospy.Service('get_flower_coords', get_flower_coords, handle_srv_call)
        print("awaiting call")
        rospy.spin()

if __name__ == "__main__":
    get_flower_coords_server()
