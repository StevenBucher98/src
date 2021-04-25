#!/usr/bin/env python3
import pyzed.sl as sl
import numpy as np
import cv2
import sys
from datetime import datetime 
from os import path

from robocrop.srv import capture_image,capture_imageResponse
import rospy


def handle_srv_call(req):
    fp = req.filepath
    if not path.exists(fp):
        return capture_imageResponse(False, "capture_image_srv/File path does not exsist")
    # Getting timestamp for saving images
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H:%M:%S")
    print("Param: ", fp)
    # ZED configuration
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K      # 2K
    init_params.camera_fps = 30                             # 30 fps
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    init_params.depth_minimum_distance = 0.1
    err = zed.open(init_params)

    if err != sl.ERROR_CODE.SUCCESS:
        return capture_imageResponse(False, "capture_image_srv/Error Opening camera")

    runtime_parameters = sl.RuntimeParameters()
    
    # IMG objects for the image and the depth video
    image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # capture the images from ZED camera
        zed.retrieve_image(image_zed, sl.VIEW.LEFT) # Get the left image
        zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)

        # conversion to opencv objs
        image_ocv = image_zed.get_data()
        depth_ocv = depth_zed.get_data()

        # saving the images
        np.savetxt(fp + "/depth_"+timestamp+".csv", depth_ocv, delimiter=",")
        if not cv2.imwrite(fp + "/image_"+timestamp+ ".png", image_ocv):
        	capture_imageResponse(False, "capture_image_srv/Error Saving Image")
        
        print("Image resolution: {0} x {1}\n".format(image_zed.get_width(), image_zed.get_height()))
        
    else:
        return capture_imageResponse(False, "capture_image_srv/Error Capturing image")

    print("Captured Image Successful")
    return capture_imageResponse(True, "capture_image_srv/Successfully captured image")

def capture_image_server():
    while(True):
        rospy.init_node('capture_image')
        s = rospy.Service('capture_image', capture_image, handle_srv_call)
        print("Capture_image awaiting call")
        rospy.spin()

if __name__ == "__main__":
    capture_image_server()
