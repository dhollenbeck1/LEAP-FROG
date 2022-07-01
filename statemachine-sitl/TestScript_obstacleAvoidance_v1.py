# -*- coding: utf-8 -*-
"""
Created on Thu Jun 30 17:05:20 2022

@author: drkfr
"""

##########################################
"""
Import libraries
"""
import VehicleClass3
import time

#import dronekit_sitl
#sitl = dronekit_sitl.start_default()
#connection_string = sitl.connection_string()
    

##########################################
"""
Declare Global and Local variables (if needed)
"""


##########################################
"""
PHASE 0: Start Program (initialization)
    Import waypoint files
    Connect to the drone
    Start key listeners
    Initialize flight parameters
    Idle until user prompts mission start
"""

LEAPFROG = VehicleClass3.UAV()
# LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True) #LEAPFROG.connect_UAV(connection_string, True)
LEAPFROG.connect_UAV('/dev/ttyAMA0',True,3)

# get current location (HOME)
LEAPFROG.get_curPosition()

# get the mission commands
num_waypoints_auto = float(LEAPFROG.download_mission())
# print("number of waypoints: " % num_waypoints_auto)
 
# set fake obstacle for testing at 2m alt
LEAPFROG.obs_lat = LEAPFROG.cur_lat + 0.00005 
LEAPFROG.obs_lon = LEAPFROG.cur_lon + 0.00005
LEAPFROG.obs_alt = LEAPFROG.cur_alt + 2 

print("Checking if waypoints are defined:")
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
# LEAPFROG.vehicle.add_attribute_listener('rangerfinder', LEAPFROG.rangefinder_callback)
# LEAPFROG.vehicle.add_attribute_listener('rangerfinder1', LEAPFROG.rangefinder1_callback)
# LEAPFROG.vehicle.add_attribute_listener('rangerfinder2', LEAPFROG.rangefinder2_callback)
# Flight params
MISSIONSTART = False

# [IDLE]
# while True:
#     userInput = input("Initialization Complete. Start Mission (y/n)?: ")
#     if userInput == 'y':
#         MISSIONSTART = True
#         print("Starting Mission!!!")    
#         time.sleep(5) # -- safety delay 
#         break
#     else:
#         break

# [TESTING]
for t in range (1,100):
    temp = LEAPFROG.getRngFinders()
    print("rng: %s" % temp)
    time.sleep(LEAPFROG.rng_Ts)
    
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
#     LEAPFROG.takeoff(LEAPFROG.takeoff_alt)
#     time.sleep(1)
#     #LEAPFROG.changeMode("LOITER")
#     #time.sleep(10)

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
# print('Entering Auto Mission')
# LEAPFROG.setMode("AUTO")
# LEAPFROG.vehicle.commands.next=1
# while True:
#     nextwaypoint=LEAPFROG.vehicle.commands.next
#     if nextwaypoint == num_waypoints_auto and LEAPFROG.distance_to_current_waypoint() < 30:
#         print('exiting auto')
#         break
#     print('Distance to waypoint (%s): %s' % (nextwaypoint, LEAPFROG.distance_to_current_waypoint()))
  
#     # if nextwaypoint==3: #Skip to next waypoint
#     #     print('Skipping to Waypoint 5 when reach waypoint 3')
#     #     vehicle.commands.next = 5
#     # if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
#     #     print("Exit 'standard' mission when start heading to final waypoint (5)")
#     #     break;
#     time.sleep(1)
    
# print('Return to GUIDED mode')
# LEAPFROG.setMode("GUIDED")

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
# LEAPFROG.vehicle.remove_message_listener('rangefinder',LEAPFROG.rangefinder_callback)


# Close connections
# if MISSIONSTART:
    # LEAPFROG.QRTL()
    
LEAPFROG.close_drone()