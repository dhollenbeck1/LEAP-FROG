"""
Created on Thursday June 9th, 2022

@author: Rafal Krzysiak

This script will test the VehicleClass script with mission planner
used for the maiden flight of LEAPFROG
"""

# -- import required libraries
import VehicleClass

# -- define the drone to be used 
LEAPFROG = VehicleClass.UAV()

# -- read waypoints txt file and define whether or not you will 
# -- implement a waypoint path from a csv file
File_exists1 = LEAPFROG.DefineWaypoints('waypoints.csv')

# -- initialize the first portion of the waypoint navigation
# -- as incomplete. This boolen will determine whether or not
# -- the drone will be safe to enter the last portion of the survey
# -- or stay on the ground and close out the drone
portion1_completed = False

# -- before even running the simulation or flying the drone
# -- check if the file exists or waypoints are defined in mission planner
if File_exists1:
    # -- connect to the drone and get the state of LEAPFROG
    # -- connect_UAV function accepts the TCP and wait_ready commands only
    LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True)

    # -- get the state of the drone
    LEAPFROG.get_state()

    # -- arm and let the UAV takeoff
    TargetAltitude = 10 # -- target altitude for initial UAV takeoff
    LEAPFROG.arm()
    LEAPFROG.takeoff(TargetAltitude)

    # -- begin the first waypoint navigation
    airspeed = 3
    groundspeed = 10
    LEAPFROG.Waypoint_Travel(airspeed, groundspeed)

    # -- mark this portion of the navigation as completed
    portion1_completed = True

# -- once the first half of the survey is completed
# -- just prior to the loiter, load up the new waypoints
File_exists2 = LEAPFROG.DefineWaypoints('waypoints.csv')

# -- before even running the simulation or flying the drone
# -- check if the file exists or waypoints are defined in mission planner
if portion1_completed and File_exists2:
    loiter_time = 5 # -- loiter time in seconds
    LEAPFROG.loiter(loiter_time)

    # -- Once the drone has completed the loiter time
    # -- Enable the drone into guided mode
    LEAPFROG.guided()

    # -- begin the next waypoint navigation
    airspeed = 3
    groundspeed = 10
    LEAPFROG.Waypoint_Travel(airspeed, groundspeed)


# -- Safely land the UAV
LEAPFROG.land()