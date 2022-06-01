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
import math

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
                print(self.wp[0]) # -- print out the waypoints
                File_exists = True

            return File_exists
        else:
            self.error_message()
            File_exists = False
            return File_exists

    def error_message(self):
        print("====================================================================================")
        print("ERROR: Pre-defined flight path not known! \n")
        print("Before we can run the simulation or fly the drone, please pre-define a flight path!")
        print("====================================================================================")

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

    # -- function that will handle the waypoint traveling
    def Waypoint_Travel(self, airspeed, groundspeed):
        print("===========================")
        print("Beginning waypoint mission.")
        print("===========================")

        # -- set the UAV's airspeed and ground speed to the defined values
        self.vehicle.airspeed = airspeed
        self.vehicle.groundspeed = groundspeed

        # -- initialize boolean for whether or not the UAV finished the waypoints
        MissionFinished = False

        # -- boolean value for transistioning to a new waypoint
        self.waypoint_met = False

        # -- begin the counter for waypoint number
        self.wp_number = 1
        self.radius = 90.0 # -- in meters

        # -- begin looping through all waypoints
        while True:
            # -- print out the coordinates of the waypoint that the UAV is travelling to
            print("Waypoint %s coordinates: %s, %s" % (self.wp_number, \
                                                        self.wp[self.wp_number][8], \
                                                        self.wp[self.wp_number][9]))

            # -- send the UAV the coordinates of the waypoint wanting to travel to
            point = LocationGlobalRelative(float(self.wp[self.wp_number][8]), \
                                            float(self.wp[self.wp_number][9]), \
                                            float(self.wp[self.wp_number][10]))
            self.vehicle.simple_goto(point, self.vehicle.groundspeed)
            
            # -- continue looping until UAV meets the waypoint
            while True:
                # -- get the distance to the waypoint
                self.distance_to_wp = self.get_distance_metres()
                print("Distance to waypoint %s: %s m" % (self.wp_number, self.distance_to_wp))

                # -- if the UAV is within the waypoint tolerance, move to the next waypoint
                if self.distance_to_wp < self.radius:
                    self.wp_number = self.wp_number + 1
                    break
                
                # -- used to slow down the printing of the distance from waypoint
                time.sleep(0.5)

            # -- check if the UAV reached the last waypoint within tolerance
            # -- if yes, end the waypoint traversal
            if self.distance_to_wp < self.radius and self.wp_number == 6:
                MissionFinished = True
                break

    # -- function for landing the UAV and closing the simulation
    def land(self):
        # -- return to initial launch site
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("QRTL")

        # -- Close vehicle object before exiting script
        self.vehicle.close()
        print("Completed")

    # -- this function will get the distance between the UAV to the waypoint
    def get_distance_metres(self):
        """
        Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        # -- calculate the dlat and dlon of the current position to the waypoint
        dlat = self.vehicle.location.global_relative_frame.lat - float(self.wp[self.wp_number][8])
        dlong = self.vehicle.location.global_relative_frame.lon - float(self.wp[self.wp_number][9])
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5