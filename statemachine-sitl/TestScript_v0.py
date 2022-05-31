"""
Created on Tuesday May 31st, 2022

@author: Rafal Krzysiak

This script will test the VehicleClass script with mission planner
"""

# -- import required libraries
import VehicleClass

# -- define the drone to be used 
LEAPFROG = VehicleClass.UAV()

# -- read waypoints txt file
LEAPFROG.DefineWaypoints('waypoints.csv')

# -- connect to the drone and get the state of LEAPFROG
# -- connect_UAV function accepts the TCP and wait_ready commands only
LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True)

# -- get the state of the drone
LEAPFROG.get_state()

# -- arm and let the UAV takeoff
TargetAltitude = 10 # -- target altitude for initial UAV takeoff
LEAPFROG.arm()
LEAPFROG.takeoff(TargetAltitude)