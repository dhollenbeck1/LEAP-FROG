"""
Created on Tuesday May 31st, 2022

@author: Rafal Krzysiak

This class will act as a communication between the
UAV (real or simulated) and dronekit/mission planner
"""

# -- import required libraries
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import os.path
import csv

# -- define a UAV class
class UAV:

    # -- define waypoints for drone
    def DefineWaypoints(self, waypointsFile):

        # -- create empty list before populating with coordinates
        self.wp = [] 

        # -- check if the waypoints file exists
        # -- if the file exists, then open the file and read the pre-defined waypoints
        if os.path.exists(waypointsFile):
            wpFile = open(waypointsFile, 'r')
            wpReader = csv.reader(wpFile, delimiter=',')

            # -- get the rows of the file and append them to a list
            for row in wpReader:
                self.wp.append(row)

            # -- inform the user that the waypoints have been defined if list is not empty   
            if self.wp is not None:
                print("Waypoints defined")
                print(self.wp) # -- print out the waypoints
        else:
            print("The waypoints file does not exist. Make sure it is in the correct directory.")

    # -- function to initialize the connection between the drone and mission planner
    def connect_UAV(self, tcp, wait_ready):
        self.vehicle = connect(tcp, wait_ready)

    # -- get the state of the UAV
    def get_state(self):
        print("Get some vehicle attribute values:")
        print(" GPS: %s" % self.vehicle.gps_0)
        print(" Battery: %s" % self.vehicle.battery)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Mode: %s" % self.vehicle.mode.name) 

    # -- function used to arm the UAV
    def arm(self):

        print("Basic pre-arm checks")

        # -- Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")

        # -- UAV should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        print("UAV successfully armed!") 

    # -- function used to make the UAV takeoff
    def takeoff(self, TargetAltitude):
        # -- As a precaution, confirm vehicle armed before attempting to take off
        print("Checking to confirm UAV has been armed successfully")
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("UAV arming confirmed. Taking off!")
        self.vehicle.simple_takeoff(TargetAltitude)  # Take off to target altitude

        # -- Wait until the vehicle reaches a safe height before processing the goto
        # -- (otherwise the command after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)

            # -- Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= TargetAltitude * 0.95:
                print("Reached target altitude")
                break

            time.sleep(1)