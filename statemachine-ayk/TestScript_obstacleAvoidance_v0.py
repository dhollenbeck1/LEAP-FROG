# -*- coding: utf-8 -*-
"""
Created on Sat Jun 11 22:03:14 2022

@author: MESA LAB
"""

##########################################
"""
Import libraries
"""
from dronekit import LocationGlobalRelative
import VehicleClass2
import time

#import dronekit_sitl
#sitl = dronekit_sitl.start_default()
#connection_string = sitl.connection_string()
    

##########################################
"""
Declare Global and Local variables
"""
global mode_currrent, mode_target
global takeoff_alt # -- target altitude for initial UAV takeoff
global oa_type

# oa_type = 2
# takeoff_alt = 20 
# mode_current = "VehicleMode:Default"
# mode_target = "VehicleMode:GUIDED"
# last_rangefinder_distance=0
# last_rangefinder1_distance=0
# last_rangefinder2_distance=0

##########################################
"""
PHASE 0: Start Program (initialization)
    Import waypoint files
    Connect to the drone
    Start key listeners
    Initialize flight parameters
    Idle until user prompts mission start
"""

LEAPFROG = VehicleClass2.UAV()

LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True) #LEAPFROG.connect_UAV(connection_string, True)

# get current location (HOME)
LEAPFROG.cur_lat = LEAPFROG.vehicle.location.global_relative_frame.lat
LEAPFROG.cur_lon = LEAPFROG.vehicle.location.global_relative_frame.lon
LEAPFROG.cur_alt = LEAPFROG.vehicle.location.global_relative_frame.alt
 
# set fake obstacle for testing at 2m alt
LEAPFROG.obs_lat = LEAPFROG.cur_lat + 0.0001 
LEAPFROG.obs_lon = LEAPFROG.cur_lon + 0.0001
LEAPFROG.obs_alt = LEAPFROG.cur_alt + 2

# print("Checking if waypoints are defined:")
# File_1_exists = LEAPFROG.DefineWaypoints('phase_1.csv')
# File_2_exists = LEAPFROG.DefineWaypoints('phase_2.csv')
# File_3_exists = LEAPFROG.DefineWaypoints('phase_3.csv')
# File_4_exists = LEAPFROG.DefineWaypoints('phase_4.csv')
# File_5_exists = LEAPFROG.DefineWaypoints('phase_5.csv')
# File_6_exists = LEAPFROG.DefineWaypoints('phase_6.csv')

print("Getting current mode")
LEAPFROG.mode_current = LEAPFROG.getMode()
time.sleep(1)
print("The current mode is: %s" % LEAPFROG.mode_current)

# add listeners
LEAPFROG.vehicle.add_attribute_listener('mode', LEAPFROG.mode_callback)
LEAPFROG.vehicle.add_attribute_listener('rangerfinder', LEAPFROG.rangefinder_callback)
# LEAPFROG.vehicle.add_attribute_listener('rangerfinder1', LEAPFROG.rangefinder1_callback)
# LEAPFROG.vehicle.add_attribute_listener('rangerfinder2', LEAPFROG.rangefinder2_callback)
# Flight params
MISSIONSTART = False

# [IDLE]
while True:
    userInput = input("Initialization Complete. Start Mission (y/n)?: ")
    if userInput == 'y':
        MISSIONSTART = True
        print("Starting Mission!!!")    
        time.sleep(5) # -- safety delay 
        break
    else:
        break
    
    
##########################################
"""
PHASE 1: Take-off and Loiter
    set takeoff params
    launch aircraft
    move to loiter point. 
"""
if MISSIONSTART == True:
    LEAPFROG.vehicle.parameters['Q_GUIDED_MODE'] = 1
    LEAPFROG.arm()
    LEAPFROG.takeoff(LEAPFROG.takeoff_alt)
    time.sleep(1)
    #LEAPFROG.changeMode("LOITER")
    #time.sleep(10)

##########################################
"""
PHASE 2:
    Move through obstacle point

"""


##########################################
"""
PHASE 3:
    ...
"""


##########################################
"""
PHASE 4:
    ...
"""


##########################################
"""
PHASE 5:
    ...
"""


##########################################
"""
PHASE 6:
    ...
"""


##########################################
"""
PHASE 7: End Program
    Remove listeners
    Close connections
"""
# Close Listeners
LEAPFROG.vehicle.remove_message_listener('mode',LEAPFROG.mode_callback)
LEAPFROG.vehicle.remove_message_listener('rangefinder',LEAPFROG.rangefinder_callback)


# Close connections
if MISSIONSTART:
    LEAPFROG.QRTL()
    
LEAPFROG.close_drone()