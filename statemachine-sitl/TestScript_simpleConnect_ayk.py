# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 17:12:49 2022

@author: MESA LAB
"""


##########################################
"""
Import libraries
"""
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

takeoff_alt = 20 
mode_current = "VehicleMode:Default"
mode_target = "VehicleMode:GUIDED"

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
LEAPFROG.connect_UAV('udp:10.49.0.4:14550', True) #LEAPFROG.connect_UAV(connection_string, True)

print("Checking if waypoints are defined:")
File_1_exists = LEAPFROG.DefineWaypoints('phase_1.csv')
File_2_exists = LEAPFROG.DefineWaypoints('phase_2.csv')
File_3_exists = LEAPFROG.DefineWaypoints('phase_3.csv')
File_4_exists = LEAPFROG.DefineWaypoints('phase_4.csv')
File_5_exists = LEAPFROG.DefineWaypoints('phase_5.csv')
File_6_exists = LEAPFROG.DefineWaypoints('phase_6.csv')

print("Getting current mode")
mode_current = LEAPFROG.getMode()
time.sleep(1)
print("The current mode is: %s" % mode_current)

# add listeners
LEAPFROG.vehicle.add_attribute_listener('mode', LEAPFROG.mode_callback)

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
    
    LEAPFROG.get_state()
##########################################
"""
PHASE 1: Take-off and Loiter
    set takeoff params
    launch aircraft
    move to loiter point. 
"""
# if MISSIONSTART == True:
#     LEAPFROG.vehicle.parameters['Q_GUIDED_MODE'] = 1
#     LEAPFROG.arm()
#     LEAPFROG.takeoff(takeoff_alt)
#     time.sleep(1)
#     #LEAPFROG.changeMode("LOITER")
#     #time.sleep(10)

##########################################
"""
PHASE 2:
    Move to control
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

# Close connections
# if MISSIONSTART == True:
#     LEAPFROG.land()
# else:      
#     LEAPFROG.close_drone()
LEAPFROG.close_drone()