import serial
import time
import inspect

def sendCommands(cmds, delay=0):
    for cmd in cmds.strip().splitlines():
        cmd = cmd.strip()
        print('Sending: ' + cmd)
        cmd = cmd + '\n' 

        s.write(cmd.encode()) # Send g-code block to grbl

        if (delay != 0):
            time.sleep(delay)

        flushGrblOutput()

def flushGrblOutput():
    while True:   
        grbl_out_raw = s.readline()
        grbl_out = grbl_out_raw.decode('utf-8').strip()
        print(grbl_out)
        if grbl_out == 'ok':
            break

def dumpSettings():
    cmd = """$$
            ?"""
    sendCommands(cmd)

def homeDevice():
    cmd = """
            $$
            $H
            G10 P0 L20 X0 Y0 Z0
            """
    sendCommands(cmd)

def mowTheLawnXY():
    cmd = ''
    for x in range(0,200,100):
        for y in range(0,400, 50):
            cmd += "G0 X{0} Y{1}\n".format(x,y)

    cmd += "G0 Y0"
    #print(cmd)
    sendCommands(cmd, delay=0.8)

def fullRangeTest():
    cmd = """
        G0 X700 Y350
        G0 Y0
        G0 X0 Y350
        G0 Y0
    """
    sendCommands(cmd, delay=1)

def jogX(amount):
    cmd = """G0 X{0}
    ?""".format(amount)

    sendCommands(cmd, delay=1)

def mowTheLawnXY2():
    cmd = ''
    for x in range(0,700,100):
        cmd += "G0 X{0} Y350\n".format(x)
        cmd += "G0 X{0} Y0\n".format(x)

    sendCommands(cmd, delay=1)

# # Open grbl serial port
# s = serial.Serial('/dev/ttyAMA0',115200)


# # Wake up grbl
# serialCmd = "\r\n\r\n"
# s.write(serialCmd.encode())
# time.sleep(2)   # Wait for grbl to initialize 
# s.flushInput()  # Flush startup text in serial input

# dumpSettings()
# homeDevice()
