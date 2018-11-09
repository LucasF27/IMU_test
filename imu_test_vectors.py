import time
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import _thread

address = 1
command = 11

portIMU = '/dev/tty.usbmodem14101' # rPi


def on_exit():
    global running, serial_port
    running = False
    serial_port.close()

app = QtGui.QApplication([])
app.aboutToQuit.connect(on_exit)
win = pg.GraphicsWindow(title="Basic plotting examples")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')
pg.setConfigOptions(antialias=True)

p6 = win.addPlot(title="Updating plot")
curve_ang = p6.plot(pen='b')
ang = [0] * 100
ptr = 0
def update():
    global curve, y, ptr, p6
    curve_ang.setData(ang[-100:-1])
    ptr = 0
    # if ptr == 0:
    #     p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(16)


serial_port = serial.Serial(port=portIMU, baudrate=115200, timeout=0.001)
time.sleep(0.1)
serial_port.flush()
time.sleep(0.1)
running = True

# Tare with current orientation
msg = '>'+str(address)+',96'+'\n'
print('Message sent: ' + msg)
serial_port.write(msg.encode())
out = ''
time.sleep(0.1)
while serial_port.inWaiting():
    out += '>> ' + serial_port.read(serial_port.inWaiting()).decode()
print('Response: ' + out)

# Turn compass off
msg = '>'+str(address)+',109,0'+'\n'
print('Message sent: ' + msg)
serial_port.write(msg.encode())
out = ''
time.sleep(0.1)
while serial_port.inWaiting():
    out += '>> ' + serial_port.read(serial_port.inWaiting()).decode()
print('Response: ' + out)

# Set euler decomposition order
msg = '>'+str(address)+',16,2'+'\n' # Possible values are 0x0 for XYZ, 0x1 for YZX, 0x2 for ZXY, 0x3 for ZYX, 0x4 for XZY or 0x5 for YXZ (default).
print('Message sent: ' + msg)
serial_port.write(msg.encode())
out = ''
time.sleep(0.1)
while serial_port.inWaiting():
    out += '>> ' + serial_port.read(serial_port.inWaiting()).decode()
print('Response: ' + out)

msg = '>'+str(address)+','+str(command)+'\n'
print('Message sent: ' + msg)

i = 0

def read_sensors(portIMU):
    global ang, running
    while running:
        serial_port.write(msg.encode())
        time.sleep(0.1)
        out = ''
        while serial_port.inWaiting():
            out += serial_port.read(serial_port.inWaiting()).decode()

        data = out.split(',')
        if len(data) > 2:
            y = -np.arcsin(np.array(data[3]).astype(float))
            if np.array(data[5]).astype(float)<0 and y>0:
                y = np.pi - y
            elif np.array(data[5]).astype(float)<0 and y<=0:
                y = -np.pi - y
            if y < 0:
                y = np.pi*2 + y
            y = y*180/np.pi
            ang.append(y)
        # print('Response: ' + out)
        # print(y)

_thread.start_new_thread(read_sensors, (portIMU, ))
# read_sensors(portIMU)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()