#!/usr/bin/env python
import sys
import rospy
from robocrop.srv import home_gantry, home_gantryResponse
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
            reset
            $X
            $$
            $H
            G10 P0 L20 X0 Y0 Z0
            """
        sc.sendCommands(cmd, s)
        # sc.homeDevice(s)

        return home_gantryResponse(True, "worked")
    return home_gantryResponse(False, "No serial port")

def home_gantry_service():
    while(True):
        rospy.init_node('home_gantry')
        serv = rospy.Service('home_gantry', home_gantry, handle_srv_call)
        print("home_gantry - awaiting call")
        rospy.spin()

if __name__ == "__main__":
    home_gantry_service()
