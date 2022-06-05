"""
Created on Tuesday May 31st, 2022

@author: Rafal Krzysiak

This script will test the VehicleClass script with mission planner
"""

# -- import required libraries
import VehicleClass

# -- define the drone to be used 
LEAPFROG = VehicleClass.UAV()

"""
Mission Phases: 
    0. Connect, get states, and adjust parameters
    1. Take-off and loiter
    2. control handoff loiter
    3. Environmental survey
"""

# -- read waypoints txt file and define whether or not you will 
# -- implement a waypoint path from a csv file
File_exists = LEAPFROG.DefineWaypoints('leap-frog_simpleauto_v3.csv')

# -- before even running the simulation or flying the drone
# -- check if the file exists or waypoints are defined in mission planner
if File_exists:
    """
    Phase : 0
    """
    # -- connect to the drone and get the state of LEAPFROG
    # -- connect_UAV function accepts the TCP and wait_ready commands only
    LEAPFROG.connect_UAV('tcp:127.0.0.1:5762', True)

    # -- get the state of the drone
    LEAPFROG.get_state()
    
    # -- set additional parameters
    LEAPFROG.vehicle.parameters['Q_GUIDED_MODE'] = 1

    # -- arm and let the UAV takeoff
    TargetAltitude = 50 # -- target altitude for initial UAV takeoff
    LEAPFROG.arm()
    LEAPFROG.takeoff(TargetAltitude)

    # -- begin waypoint navigation
    airspeed = 3
    groundspeed = 10
    LEAPFROG.Waypoint_Travel(airspeed, groundspeed)

    # -- Safely land the UAV
    LEAPFROG.land()