# -*- coding: utf-8 -*-
"""
Created on Sun Jun  5 00:16:17 2022

@author: MESA LAB
"""
import VehicleClass
import time

def location_callback(self, attr_name, value):
     print("Location (Global): ", value)

LEAPFROG = VehicleClass.UAV()

LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True)

@LEAPFROG.vehicle.on_attribute('mode')
def mode_callback(self,attr_name, value):
     #attr_name not used here.
     print(" mode : %s" % self.mode.name)
     
#LEAPFROG.vehicle.add_attribute_listener('location.global_frame', location_callback)

# last_rangefinder_distance = 0
# @LEAPFROG.vehicle.on_attribute('rangefinder')
# def rangefinder_callback(self,attr_name):
#      #attr_name not used here.
#      global last_rangefinder_distance
#      if last_rangefinder_distance == round(self.rangefinder.distance, 1):
#          return
#      last_rangefinder_distance = round(self.rangefinder.distance, 1)
#      print(" Rangefinder (metres): %s" % last_rangefinder_distance)


time.sleep(2)

LEAPFROG.arm()
TargetAltitude = 10
LEAPFROG.takeoff(TargetAltitude)

time.sleep(1)

LEAPFROG.land()
#LEAPFROG.close_drone()