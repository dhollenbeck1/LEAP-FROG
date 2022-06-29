"""
Created on Thursday June 9th, 2022

@author: Rafal Krzysiak

This script will test the VehicleClass script with mission planner
used for the maiden flight of LEAPFROG.

The dronekit maiden flight test is a single phase survey however, 
the single phase is broken up into sections.
The drone will take off and then begin a waypoint navigation
Once completing a portion of the waypoint navigation, the drone will then
be placed into loiter mode for some time. After the time is 
"""

# -- import required libraries
import VehicleClass3
import time

# -- define the drone to be used 
LEAPFROG = VehicleClass3.UAV()

# -- connect to the drone and get the state of LEAPFROG
# -- connect_UAV function accepts the TCP and wait_ready commands only
LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True)

# -- get the state of the drone
LEAPFROG.get_state()

# -- arm and let the UAV takeoff
TargetAltitude = 10 # -- target altitude for initial UAV takeoff
LEAPFROG.getMode()
LEAPFROG.arm()
LEAPFROG.takeoff(TargetAltitude)
time.sleep(10) # -- let the drone hover in place at target altitude for 10 seconds

# -- Safely land the UAV
LEAPFROG.land()