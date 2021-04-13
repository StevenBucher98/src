#!/usr/bin/env python3
import pyzed.sl as sl
import numpy as np
import cv2
import sys
import rospy

from camera.srv import get_flower_coords, get_flower_coordsResponse
from camera.msg import Coord

def handle_srv_call(req):
    # setting up ZED Camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K      # 2K
    init_params.camera_fps = 30                             # 30 fps
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    init_params.depth_minimum_distance = 0.1
    err = zed.open(init_params)

    if err != sl.ERROR_CODE.SUCCESS:
        return get_flower_coordsResponse([], "get_flower_coords/Error Opening camera")

    runtime_parameters = sl.RuntimeParameters()
    # IMG objects for the image and the depth video
    image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    
    res = []
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # capture the images from ZED camera
        zed.retrieve_image(image_zed, sl.VIEW.LEFT) # Get the left image
        zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)

        # conversion to opencv objs
        img = image_zed.get_data()
        depth_ocv = depth_zed.get_data()

        # Convert to HSV and Greyscale respectively
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # HSV value range
        hsv_color1 = np.asarray([0, 0, 255])
        hsv_color2 = np.asarray([255, 20, 255])
        
        # Filter out the specific colors from above
        mask = cv2.inRange(hsv_img, hsv_color1, hsv_color2)
        res = cv2.bitwise_and(img, img, mask=mask)

        # Kernals for morphological transformations
        kernel = np.ones((5, 5), np.uint8)
        big_kernal = np.ones((50, 50), np.uint8)

        # initally opening of the result of the filtered colors
        opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)

        # dilation with small kernal
        dilation = cv2.dilate(opening, kernel, iterations=10)

        # bigger kernal closing 
        final = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, big_kernal)

        # convert to greyscale for finding contours
        final_gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
        im2, contours, hierarchy = cv2.findContours(final_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        print("Number of Contours: ", len(contours))
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            print("X Y W H")
            print(x, y, w, h)
            print("________________")
            cv2.rectangle(img, (x, y), (x+w, y+h), (255,0,0), 10)
            
            # Gets the average
            Z = np.mean(depth_ocv[x:x+w][y:y+h])

            # TODO
            # tranform the X and Y into positions that are more relvative to the gantry

            temp = Coord()
            temp.x = x
            temp.y = y
            temp.z = Z
            res.append(temp)

    else:
        return get_flower_coordsResponse([], "get_flower_coords/Error Capturing image")

    return get_flower_coordsResponse(res, "get_flower_coords/success")

def get_flower_coords_server():
    while(True):
        rospy.init_node('get_flower_coords')
        s = rospy.Service('get_flower_coords', get_flower_coords, handle_srv_call)
        print("get flower coords - awaiting call")
        rospy.spin()

if __name__ == "__main__":
    get_flower_coords_server()
