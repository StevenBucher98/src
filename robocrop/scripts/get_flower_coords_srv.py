#!/usr/bin/env python3
import pyzed.sl as sl
import numpy as np
import cv2
import sys
import rospy
import math
from datetime import datetime 
from robocrop.srv import get_flower_coords, get_flower_coordsResponse
from robocrop.msg import Coord
import time
X_SCALAR = 0.45289
Y_SCALAR = 0.44843
X_OFFSET = 235#mm 224 was previous good one 
Y_OFFSET = 135#mm 127
                

def handle_srv_call(req):
    print("service called")
    # setting up ZED Camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K      # 2K
    init_params.camera_fps = 30                             # 30 fps
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    init_params.depth_minimum_distance = 0.1
    err = zed.open(init_params)
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H:%M:%S")
    start = time.time()
    if err != sl.ERROR_CODE.SUCCESS:
        return get_flower_coordsResponse([], "get_flower_coords/Error Opening camera")

    runtime_parameters = sl.RuntimeParameters()
	# IMG objects for the image and the depth video
    image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    
	# Resulting flowers
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
        kernel = np.ones((3, 3), np.uint8)
        big_kernal = np.ones((20, 20), np.uint8)

        # initally opening of the result of the filtered colors
        opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        # dilation with small kernal
        #dilation = cv2.dilate(opening, kernel, iterations=10)

        # bigger kernal closing 
        #final = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, big_kernal)

        final = closing

        # convert to greyscale for finding contours
        final_gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(final_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        result_contours = []
        print("contor len", len(contours))
        flower_count = 0
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = w*h
            if area > 1500  and y > 0 and x > 0 and y < 1000 and x < 1500:
            
                cv2.rectangle(img, (x, y), (x+w, y+h), (0,255,0), 10)
                cv2.putText(img, "flower " + str(flower_count), (x-20,y), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255,0), 3, cv2.LINE_AA)

				# get centroid 
                centroid = (int(x+(w/2)), int(y+(h/2)))
               	
                # Gets the average z depth
                sum_ = 0
                count_ = 0
                for x_i in range(x, x+w):
                    for y_i in range(y, y+h):
                        v = depth_ocv[y_i][x_i]
                        if not np.isnan(v):
                            count_ +=1                
                            sum_ += v  
                if sum_ != 0 or count_ !=0:            
                    ave = sum_/count_
                else:
                    ave = -1
               
                print("Flower ", flower_count)			
                print("Pixel Coord: (X Y W H)")
                print(x, y, w, h)
                print("centroid: ", centroid)
                print("scaled Cent: ", (centroid[0] * X_SCALAR, centroid[1] * Y_SCALAR))
                
                scaled_x = (centroid[0] * X_SCALAR) - X_OFFSET + 27
                scaled_y = (centroid[1] * Y_SCALAR) + Y_OFFSET + 17
                
                # making sure we do not go in negative
                if scaled_x < 0:
                    scaled_x = 0
                print("Offset Cent: ", (scaled_x, scaled_y))
                print("-------------------")
                
                flower = Coord() # Coord is my predetermined ROS Message to help with transfering a dynamic list of coords
                flower.x = scaled_x
                flower.y = scaled_y
                flower.z = ave

                flower_count += 1
                result_contours.append(flower)

        print("finding image time: ", abs(start - time.time()), "s")	
        start = time.time()
        cv2.imwrite('/home/imagebay/Demo_test_image'+timestamp+ '.png', img)
        print("Save image time: ", abs(start -time.time()), "s")
    else:
        return get_flower_coordsResponse([], "get_flower_coords/Error Capturing image")

    return get_flower_coordsResponse(result_contours, "get_flower_coords/success")

def get_flower_coords_server():
    while(True):
        rospy.init_node('get_flower_coords')
        s = rospy.Service('get_flower_coords', get_flower_coords, handle_srv_call)
        print("get flower coords - awaiting call")
        rospy.spin()

if __name__ == "__main__":
    get_flower_coords_server()
