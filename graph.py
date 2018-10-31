import serial
import time
import binascii
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import re
import _thread

def on_exit():
    global running, counters, addresses, serial_port
    running = False
    for i in range(len(addresses)):
        print('f' + str(i + 1) + ' = ' + str(counters[i] / time_to_measure))
    for i in range(len(addresses)):
        serial_port.write(('>'+str(addresses[i])+',86\n').encode())
    serial_port.close()

app = QtGui.QApplication([])
app.aboutToQuit.connect(on_exit)
win = pg.GraphicsWindow(title="Basic plotting examples")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')
pg.setConfigOptions(antialias=True)

p6 = win.addPlot(title="Updating plot")
curve_x = p6.plot(pen='y')
curve_y = p6.plot(pen='b')
curve_z = p6.plot(pen='g')
x = [3.5] * 1000
y = [-3.5] * 1000
z = [0] * 1000
ptr = 0
def update():
    global curve, x, ptr, p6
    curve_x.setData(x[-1000:-1])
    curve_y.setData(y[-1000:-1])
    curve_z.setData(z[-1000:-1])
    ptr = 0
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

number_of_sensors = 1
p = False
time_to_measure = 10
command = 1
running = True

addresses = [2,8,2,3,4,5,1]
counters = [0] * len(addresses)

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
    global x,y,z, running, counters
    t0 = time.time()

    while running:

        bytes_to_read = serial_port.inWaiting()

        if bytes_to_read > 0:
            data = serial_port.read(bytes_to_read)
            data = data.decode().replace('\r\n',' ')
            data2 = ''.join(chr(i) for i in data.encode() if ord(chr(i)) > 31 and ord(chr(i)) < 128 )
            data3 = data2.split(' ')
            data3 = list(filter(None, data3))
            # print(data3)

            new_elements = len(data3)
            for i in range(0, new_elements):
                new_data = data3[i].split(',')
                new_data = np.array(new_data).astype(np.float)
                x.append(new_data[0])
                y.append(new_data[1])
                z.append(new_data[2])

            # print(x,y,z)
            counters[0] += new_elements
            # print(x[-1])
        else:
            # print("No data")
            # time.sleep(0.1)
            pass





_thread.start_new_thread(read_sensors, (portIMU, ))
# read_sensors(portIMU)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()