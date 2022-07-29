# -*- coding: utf-8 -*-
"""
Created on Thu July 28 2022

@author: Di An
"""

##########################################
"""
Import libraries
"""
#import VehicleClass3
from math import pi,sqrt,sin,cos,atan2,radians
from geographiclib.geodesic import Geodesic
# from sys import platform
# from os.path import join
#from imp import load_source
import os
import sys
import pynmea2
import io
import time

#init parameters
current_position = [37.3747196, -120.5778529, 0] #Castle GPS location
#desired_position = []
take_off_alt = 50
flight_velocity = 2 #2m/s
delta_t = 1 # 1s
desired_distance = 100
path = os.path.abspath(os.path.dirname(sys.argv[0]))
#Take off alt = 50
gps_matrix = []

def take_off(alt):
    while current_position[2] is not alt:
        current_position[2] = current_position[2] + flight_velocity*delta_t
        print(current_position[2])
        gps_matrix.extend(current_position)



# calculate distance based on two GPS coordinates, using Haversine function
def adding_distance():
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    # lat, long, azimuth, the distance from the first point to the second in meters
    # Direct(lat,long,azimuth,distance)
    distance = 2
    delta_distance = 2
    while distance is not desired_distance:
        g = geod.Direct(current_position[0], current_position[1], 225, delta_distance)
        #print("The position is ({:.8f}, {:.8f}).".format(g['lat2'],g['lon2']))

        gps_matrix.extend(current_position) 
        current_position[0] = round(g['lat2'],7)
        current_position[1] = round(g['lon2'],7)        
        # with open(path+'/'+'GPS'+'.txt',"w") as w:
        #     w.writelines(str(gps_matrix[0])+","+str(gps_matrix[1])+","+str(current_position[2])+"\n")
        distance = distance + 2 * delta_t
        #print(distance)
        print(current_position)
        #print(gps_matrix)
    #print(gps_matrix)
    return gps_matrix

def write_func(gps):
    with open(path+'/'+'GPS'+'.txt',"w") as w:
        for i in range(0,len(gps),3):
            w.write(str(gps[i])+","+str(gps[i+1])+","+str(gps[i+2])+"\n")
            #for i in range(len(gps_matrix)):

def gps2NMEA(gps):
    with open(path+'/'+'NEMA'+'.txt',"w") as w:
        for i in range(0,len(gps),3):
            msg = pynmea2.GGA('GP','GGA',('',str(gps[i]),'N',str(abs(gps[i+1])),'W','1','04','2.6',str(gps[i+2]),'M','-33.9','M','','0000'))
            w.write(str(msg)+"\n")

if __name__ == '__main__':
    take_off(take_off_alt)
    gps = adding_distance()
    write_func(gps)
    gps2NMEA(gps)
