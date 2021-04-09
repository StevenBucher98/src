#!/usr/bin/env python3
import pyzed.sl as sl
import numpy as np
import cv2
import sys

from cartesian_control.srv import test_srv,test_srvResponse
import rospy
import random
import RPi.GPIO as GPIO

def handle_srv_call(req):
        # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
        print("Cartesian Robot Called!")
        zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD2K      # 2K
        init_params.camera_fps = 30                             # 30 fps
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.MILLIMETER
        init_params.depth_minimum_distance = 0.1
        err = zed.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            print("error Opening camera")
            return test_srcResponse("Camera failed", False)
#        now = datetime.now()

 #       timestamp = now.strftime("%Y-%m-%d_%H:%M:%S")

        image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
            print("Image resolution: {0} x {1}\n".format(image.get_width(), image.get_height()))

        print("move the robot here.....")
        return test_srvResponse("Successfully Called", True)

def test_srv_server():
        while(True):
                rospy.init_node('test_srv_server')
                s = rospy.Service('test_srv', test_srv, handle_srv_call)
                print("awaiting call")
                rospy.spin()

if __name__ == "__main__":
    test_srv_server()
