# -*- coding: utf-8 -*-
"""
Created on Sun Jun  5 08:46:38 2022

@author: MESA LAB (Rafal, Derek, Di)
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
PHASE 0: Start Program (initialization)
    Import waypoint files
    Connect to the drone
    Start key listeners
    Initialize flight parameters
    Idle until user prompts mission start
"""
LEAPFROG = VehicleClass2.UAV()
LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True) #LEAPFROG.connect_UAV(connection_string, True)

File_1_exists = LEAPFROG.DefineWaypoints('phase_1.csv')
File_2_exists = LEAPFROG.DefineWaypoints('leap-frog_simpleauto_v3.csv')

global mode_currrent
mode_current = LEAPFROG.vehicle.mode.name
print(mode_current)

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
    TargetAltitude = 20 # -- target altitude for initial UAV takeoff
    LEAPFROG.arm()
    LEAPFROG.takeoff(TargetAltitude)
    time.sleep(1)
    #LEAPFROG.changeMode("LOITER")
    #time.sleep(10)

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
if MISSIONSTART == True:
    LEAPFROG.land()
else:      
    LEAPFROG.close_drone()