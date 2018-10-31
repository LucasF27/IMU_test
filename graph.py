import serial
import time
import binascii
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np

number_of_sensors = 1
p = False
time_to_measure = 10
command = 1

addresses = [2,8,2,3,4,5,1]

portIMU = '/dev/tty.usbmodem14101' # rPi
serial_port = serial.Serial(port=portIMU, baudrate=115200, timeout=0.001)
time.sleep(0.1)
serial_port.flush()
time.sleep(0.1)

# Set streaming slots
for i in range(len(addresses)):
    msg = '>'+str(addresses[i])+',80,'+str(command)+',255,255,255,255,255,255,255\n'
    print(msg)
    serial_port.write(msg.encode())
    time.sleep(0.1)
    out = ''
    while serial_port.inWaiting():
        out += '>> ' + serial_port.read(serial_port.inWaiting()).decode()
    print(out)
out = ''

# Start streaming
for i in range(len(addresses)):
    serial_port.write(('>'+str(addresses[i])+',85\n').encode())
    time.sleep(0.1)
    while serial_port.inWaiting():
        out = '>> ' + serial_port.read(serial_port.inWaiting()).decode()

print('Start')

def read_sensors(portIMU):
    counters = [0] * len(addresses)
    t0 = time.time()

    while 1:
        if time.time()-t0 >= time_to_measure:

            for i in range(len(addresses)):
                print('f'+str(i+1)+' = ' + str(counters[i] / time_to_measure))
            break

        bytes_to_read = serial_port.inWaiting()

        if bytes_to_read > 0:
            data = bytearray(serial_port.read(bytes_to_read)).decode()

            # angle
            b = ''.join(chr(i) for i in data[8:12])  # angle x
            ang = struct.unpack('>f', b)
            x = ang[0]
            # print(x)

            # angle
            b = ''.join(chr(i) for i in data[12:16])  # angle y
            ang = struct.unpack('>f', b)
            y = ang[0]

            # angle
            b = ''.join(chr(i) for i in data[16:20])  # angle z
            ang = struct.unpack('>f', b)
            z = ang[0]

            print(x,y,z)
            counters[0] += 1
        else:
            print("No data")

    for i in range(len(addresses)):
        serial_port.write(('>'+str(addresses[i])+',86\n').encode())


    serial_port.close()

read_sensors(portIMU)