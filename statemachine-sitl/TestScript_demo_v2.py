# -*- coding: utf-8 -*-
"""
Created on Sat Jul  2 22:15:23 2022

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
LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True,1) #LEAPFROG.connect_UAV(connection_string, True)
# LEAPFROG.connect_UAV('/dev/ttyS0',True,2)

# get current location (HOME)
LEAPFROG.get_curPosition()

# get the mission commands
num_waypoints_auto = float(LEAPFROG.download_mission())
print("number of waypoints: %s" % num_waypoints_auto)
 

# print("Checking if waypoints are defined:")
# File_1_exists = LEAPFROG.DefineWaypoints('phase_1.csv')
# File_2_exists = LEAPFROG.DefineWaypoints('phase_2.csv')
# File_3_exists = LEAPFROG.DefineWaypoints('phase_3.csv')
# File_4_exists = LEAPFROG.DefineWaypoints('phase_4.csv')
# File_5_exists = LEAPFROG.DefineWaypoints('phase_5.csv')
# File_6_exists = LEAPFROG.DefineWaypoints('phase_6.csv')

# set fake obstacle for testing at 2m alt
LEAPFROG.OV_enable_flag = 0
LEAPFROG.obs_lat = LEAPFROG.cur_lat + 0.00005 
LEAPFROG.obs_lon = LEAPFROG.cur_lon + 0.00005
LEAPFROG.obs_alt = LEAPFROG.cur_alt + 2 

print("Getting current mode")
LEAPFROG.mode_current = LEAPFROG.getMode()
time.sleep(1)
print("The current mode is: %s" % LEAPFROG.mode_current)

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

# [TESTING]
# for t in range (1,100):
#     temp = LEAPFROG.getRngFinders()
#     print("rng: %s" % temp)
#     time.sleep(LEAPFROG.rng_Ts)
    
##########################################
"""
PHASE 1: 
    Take off, Transition, and Loiter for Observers
"""
if MISSIONSTART:
    LEAPFROG.vehicle.parameters['Q_GUIDED_MODE'] = 1
    LEAPFROG.arm()
    LEAPFROG.takeoff(LEAPFROG.takeoff_alt)
    time.sleep(1)
    File_0_exists = LEAPFROG.DefineWaypoints('phase_1_demo_v0.csv')
    # File_1_exists = LEAPFROG.DefineWaypoints('phase_n_example.csv')
    LEAPFROG.Waypoint_Travel(22)  # airspeed
    #LEAPFROG.changeMode("LOITER")
    #time.sleep(10)
    LEAPFROG.setMode("LOITER")
    loiterTime = 1  # min
    loiterTime_interval = 20 # sec
    while True:
        kmax = int((60*loiterTime/loiterTime_interval))
        for k in range (1,kmax):
            timeLeft = (60*loiterTime-k*loiterTime_interval)
            print("Time left in LOITER: %s " % timeLeft)
            time.sleep(loiterTime_interval)      
        break
    
    LEAPFROG.setMode("GUIDED")
##########################################
"""
PHASE 2:
    Travel to Control Handoff point 1 from Takeoff location
"""
if MISSIONSTART:
    print("PHASE 2: Moving to control handoff point")
    print("defining PHASE 2 waypoints:")
    File_2_exists = LEAPFROG.DefineWaypoints('phase_2_demo_v2.csv')
    # File_1_exists = LEAPFROG.DefineWaypoints('phase_n_example.csv')
    LEAPFROG.Waypoint_Travel(22)  # airspeed


##########################################
"""
PHASE 3:
    Control handoff to Environmental Survey Leg
"""
if MISSIONSTART:
    print("PHASE 3: Control handoff point 1")
    print("Switching mode to: LOITER")
    # LEAPFROG.vehicle.parameters['Q_GUIDED_MODE'] = 0
    LEAPFROG.setMode("LOITER")
    loiterTime = 1  # min
    loiterTime_interval = 20 # sec
    while True:
        kmax = int((60*loiterTime/loiterTime_interval))
        for k in range (1,kmax):
            timeLeft = (60*loiterTime-k*loiterTime_interval)
            print("Time left in LOITER: %s " % timeLeft)
            time.sleep(loiterTime_interval)      
        break

    # LEAPFROG.setMode("GUIDED")
##########################################
"""
PHASE 4:
    Environmental Survey Leg
"""
if MISSIONSTART:
    print('PHASE 4: Entering Auto Mission')
    print("Switching mode to: Auto")
    LEAPFROG.setMode("AUTO")
    LEAPFROG.vehicle.commands.next=1
    while True:
        nextwaypoint=LEAPFROG.vehicle.commands.next
        # print(num_waypoints_auto)
        # if (nextwaypoint == (num_waypoints_auto)) and (LEAPFROG.distance_to_current_waypoint() < 100):
        if (nextwaypoint == int(num_waypoints_auto-1)):
            print('exiting auto')
            break
        
        print('Distance to waypoint (%s): %s' % (nextwaypoint, LEAPFROG.distance_to_current_waypoint()))     
        # if nextwaypoint==3: #Skip to next waypoint
        #     print('Skipping to Waypoint 5 when reach waypoint 3')
        #     vehicle.commands.next = 5
        # if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        #     print("Exit 'standard' mission when start heading to final waypoint (5)")
        #     break;
        time.sleep(1)
        
   
    print('Return to GUIDED mode')
    LEAPFROG.setMode("GUIDED")
 # time.sleep(5)
##########################################
"""
PHASE 5:
    Travel to Control Handoff point 2: Mobile Leg
"""
if MISSIONSTART:
    print("PHASE 5: Moving to control handoff point 2")
    print("defining PHASE 5 waypoints:")
    File_1_exists = LEAPFROG.DefineWaypoints('phase_5_demo_v2.csv')
    LEAPFROG.Waypoint_Travel(22) # airspeed


##########################################
"""
PHASE 6:
    Control Handoff to Mobile Leg
"""
if MISSIONSTART:
    print("PHASE 6: Control handoff point 2")
    print("Switching mode to: LOITER")
    LEAPFROG.setMode("LOITER")
    loiterTime = 1  # min
    loiterTime_interval = 20
    print("LOITER time: %s, sleep interval: %s" % (loiterTime, loiterTime_interval))
    while True:
        kmax = int(60*loiterTime/loiterTime_interval)
        for k in range (1,kmax):
            timeLeft = (60*loiterTime-k*loiterTime_interval)
            print("Time left in LOITER: %s " % timeLeft)
            time.sleep(loiterTime_interval)
            
        break

    LEAPFROG.setMode("GUIDED")
##########################################
"""
PHASE 7:
    Mobile Leg
"""
if MISSIONSTART:
    print("PHASE 7: Mobile Leg")
    print("defining PHASE 7 waypoints:")
    File_1_exists = LEAPFROG.DefineWaypoints('phase_7_demo_v0.csv')
    LEAPFROG.Waypoint_Travel(22) # airspeed

##########################################

"""
PHASE 8:
    Control Handoff point 3: Cruising Leg.
"""
if MISSIONSTART:
    print("PHASE 8: Control Handoff point 3: Cruising Leg")
    print("Switching mode to: LOITER")
    LEAPFROG.setMode("LOITER")
    loiterTime = 1  # min
    loiterTime_interval = 20
    print("LOITER time: %s, sleep interval: %s" % (loiterTime, loiterTime_interval))
    while True:
        kmax = int(60*loiterTime/loiterTime_interval)
        for k in range (1,kmax):
            timeLeft = (60*loiterTime-k*loiterTime_interval)
            print("Time left in LOITER: %s " % timeLeft)
            time.sleep(loiterTime_interval)
            
        break

    LEAPFROG.setMode("GUIDED")
##########################################

"""
PHASE 9:
    Travel to Cruising Leg and Loiter (Cruising Leg)
"""
if MISSIONSTART:
    print("PHASE 9: Travel to Cruising Leg and Loiter (Cruising Leg)")
    print("defining PHASE 9 waypoints:")
    File_1_exists = LEAPFROG.DefineWaypoints('phase_9_demo_v0.csv')
    LEAPFROG.Waypoint_Travel(22) # airspeed
    print("Switching mode to: LOITER")
    LEAPFROG.setMode("LOITER")
    loiterTime = 1  # min
    loiterTime_interval = 20
    print("LOITER time: %s, sleep interval: %s" % (loiterTime, loiterTime_interval))
    while True:
        kmax = int(60*loiterTime/loiterTime_interval)
        for k in range (1,kmax):
            timeLeft = (60*loiterTime-k*loiterTime_interval)
            print("Time left in LOITER: %s " % timeLeft)
            time.sleep(loiterTime_interval)
            
        break

    LEAPFROG.setMode("GUIDED")
##########################################

"""
PHASE 10:
    Landing Sequence
"""
if MISSIONSTART:
    print("PHASE 10: QRTL and Landing sequence")

# Close Listeners
LEAPFROG.vehicle.remove_message_listener('mode',LEAPFROG.mode_callback)
# LEAPFROG.vehicle.remove_message_listener('rangefinder',LEAPFROG.rangefinder_callback)

# Landing sequence if mission started
if MISSIONSTART:
    LEAPFROG.QRTL()
    
# Close connections 
LEAPFROG.close_drone()
##########################################