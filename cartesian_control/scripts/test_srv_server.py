#!/usr/bin/env python

from __future__ import print_function

from cartesian_control.srv import test_srv,test_srvResponse
import rospy
import random
import RPi.GPIO as GPIO

def handle_srv_call(req):
	# print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
	print("Cartesian Robot Called!")
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