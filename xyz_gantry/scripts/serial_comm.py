import serial
import time
import inspect

def sendCommands(cmds, s, delay=0):
    for cmd in cmds.strip().splitlines():
        cmd = cmd.strip()
        print('Sending: ' + cmd)
        cmd = cmd + '\n' 

        s.write(cmd.encode()) # Send g-code block to grbl

        if (delay != 0):
            time.sleep(delay)

        flushGrblOutput(s)

def flushGrblOutput(s):
    while True:   
        grbl_out_raw = s.readline()
        grbl_out = grbl_out_raw.decode('utf-8').strip()
        print(grbl_out)
        if grbl_out == 'ok':
            break

def dumpSettings(s):
    cmd = """$$
            ?"""
    sendCommands(cmd, s)

def homeDevice(s):
    cmd = """
            $$
            $H
            G10 P0 L20 X0 Y0 Z0
            """
    sendCommands(cmd, s)

def mowTheLawnXY(s):
    cmd = ''
    for x in range(0,200,100):
        for y in range(0,400, 50):
            cmd += "G0 X{0} Y{1}\n".format(x,y)

    cmd += "G0 Y0"
    #print(cmd)
    sendCommands(cmd, s,  delay=0.8)

def fullRangeTest(s):
    cmd = """
        G0 X700 Y350
        G0 Y0
        G0 X0 Y350
        G0 Y0
    """
    sendCommands(cmd,s, delay=1)

def jogX(amount, s):
    cmd = """G0 X{0}
    ?""".format(amount)

    sendCommands(cmd,s, delay=1)


# # Open grbl serial port
# s = serial.Serial('/dev/ttyAMA0',115200)


# # Wake up grbl
# serialCmd = "\r\n\r\n"
# s.write(serialCmd.encode())
# time.sleep(2)   # Wait for grbl to initialize 
# s.flushInput()  # Flush startup text in serial input

# dumpSettings()
# homeDevice()
