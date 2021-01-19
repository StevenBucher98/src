#!/usr/bin/env python

from __future__ import print_function

from controller.srv import relaycontrol,relaycontrolResponse
import rospy
import random
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)
relay1_p = 3 #GPIO2
relay2_p = 5 #GPIO4
GPIO.setup(relay1_p, GPIO.OUT) 
GPIO.setup(relay2_p, GPIO.OUT) 


def handle_srv_call(req):
    # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    print("----------------------------------------")

    print("Relay Service Called from :", req.caller)
    print("Turning Relay 1 to ", req.relay1)
    res = [0,0]
    if(req.relay1):
        GPIO.output(relay1_p, GPIO.HIGH)
        res[0] = 1
    else:
        GPIO.output(relay1_p, GPIO.LOW)
        res[0] = 0


    print("Turning Relay 2 to ", req.relay2)
    if(req.relay2):
        GPIO.output(relay2_p, GPIO.HIGH)
        res[1] = 1

    else:
        GPIO.output(relay2_p, GPIO.LOW)
        res[1] = 0
    print("----------------------------------------")

    return relaycontrolResponse(res)

def test_srv_server():
	while(True):
		rospy.init_node('test_srv_server')
		s = rospy.Service('relaycontrol', relaycontrol, handle_srv_call)
		print("awaiting call")
		rospy.spin()

if __name__ == "__main__":
    test_srv_server()