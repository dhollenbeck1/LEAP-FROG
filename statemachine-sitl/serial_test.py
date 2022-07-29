import io

import pynmea2
import serial
import os
import sys
import time

path = os.path.abspath(os.path.dirname(sys.argv[0]))

ser = serial.Serial('/dev/cu.usbserial-AM021O5G', 9600, parity=serial.PARITY_NONE, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
#sio = io.TextIOBase(ser)
nema = []
#nema_matrix = []
with open(path+'/'+'NEMA'+'.txt',"r") as r:
    nema = r.readlines()
    #print(nema)
    

i = 0

while 1:
    if i == 74:
        i = 0   
    #print(nema[i])
    #print(nema[i].encode('UTF-8'))
    #line = ser.write(nema[i].encode('UTF-8'))
    line = ser.read()
    #msg = pynmea2.parse(line)
    i = i + 1
    time.sleep(0.2)
        #msg = pynmea2.parse(line)
    print(line)
    #print(repr(line))
    #print(msg)