#!/usr/bin/env python
import sys
import rospy
from xyz_gantry.srv import move_gantry, move_gantryResponse
import serial
import serial_comm as sc
import time

def handle_srv_call(req):
    with serial.Serial('/dev/ttyAMA0', 115200, timeout=5) as s:
        serialCmd = "\r\n\r\n"
        s.write(serialCmd.encode())
        time.sleep(2)
        s.reset_input_buffer() # flushInput() in python 3 <
        sc.dumpSettings(s)

        cmd = """
            $$
            G0 X""" + str(req.x) + " Y" + str(req.y) +" Z" + str(req.z) + " F" +str(req.f) +"""
            """
        sc.sendCommands(cmd, s)
        return move_gantryResponse(True, "worked")
    return move_gantryResponse(False, "No serial port")

def move_gantry_service():
    while(True):
        rospy.init_node('move_gantry')
        serv = rospy.Service('move_gantry', move_gantry, handle_srv_call)
        print("move_gantry - awaiting call")
        rospy.spin()

if __name__ == "__main__":
    move_gantry_service()
