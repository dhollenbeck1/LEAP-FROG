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

            # -- we want to know the number of rows that are present in the waypoint file
            self.num_wps = 0

            # -- get the rows of the file and append them to a list
            for row in wpReader:
                self.wp.append(row)
                self.num_wps = self.num_wps + 1

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

        self.setMode("GUIDED")
        self.vehicle.parameters['Q_GUIDED_MODE'] = 1
        self.vehicle.armed = True

        print("UAV successfully armed!") 

    # -- function used to make the UAV takeoff
    def takeoff(self, TargetAltitude):
        # -- As a precaution, confirm vehicle armed before attempting to take off
        print("Checking to confirm UAV has been armed successfully")
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            #self.vehicle.armed = True
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
        self.wp_number = 0
        self.radius = 90.0 # -- in meters
        self.vehicle.parameters['Q_WP_RADIUS'] = 10
        self.kr = 0.0002

        # -- begin looping through all waypoints
        while True:
            # -- print out the coordinates of the waypoint that the UAV is travelling to
            print("Waypoint %s coordinates (lat, lon): %s, %s" % (self.wp_number, \
                                                        self.wp[self.wp_number][8], \
                                                        self.wp[self.wp_number][9]))

            # -- in this instance of the function call, we are using the other values
            # -- self.dlat, self.dlong and self.norm to calcuate a virtual point for the 
            # -- drone to fly to so that the drone does not slow down in the air, This reduces battery waste
            self.get_distance_metres()
            self.virtual_wp = (float(self.wp[self.wp_number][8]) + (self.dlat + self.kr*(self.dlat/self.norm)),\
                               float(self.wp[self.wp_number][9]) + (self.dlong + self.kr*(self.dlong/self.norm)))

            #print(len(self.virtual_wp))

            # -- send the UAV the coordinates of the waypoint wanting to travel to
            #point = LocationGlobalRelative(float(self.wp[self.wp_number][8]), \
            #                                float(self.wp[self.wp_number][9]), \
            #                                float(self.wp[self.wp_number][10]))

            # -- send the UAV the coordinates of the waypoint wanting to travel to
            point = LocationGlobalRelative(self.virtual_wp[0], \
                                            self.virtual_wp[1], \
                                            float(self.wp[self.wp_number][10]))                                
            self.vehicle.simple_goto(point, self.vehicle.groundspeed)

            # -- previous distance
            self.previous_dist = self.distance_to_wp
            
            # -- continue looping until UAV meets the waypoint
            while True:
                # -- get the distance to the waypoint
                self.previous_dist = self.distance_to_wp

                # -- used to slow down the printing of the distance from waypoint
                time.sleep(0.25)

                self.get_distance_metres()
                self.virtual_wp = (float(self.wp[self.wp_number][8]) + (self.dlat + self.kr*(self.dlat/self.norm)),\
                                   float(self.wp[self.wp_number][9]) + (self.dlong + self.kr*(self.dlong/self.norm)))

                # -- send the UAV the coordinates of the waypoint wanting to travel to
                point = LocationGlobalRelative(self.virtual_wp[0], \
                                               self.virtual_wp[1], \
                                               float(self.wp[self.wp_number][10]))    

                self.vehicle.simple_goto(point, self.vehicle.groundspeed)
                print("Distance to waypoint %s: %s m" % (self.wp_number, self.distance_to_wp))

                # -- if the UAV is within the waypoint tolerance, move to the next waypoint
                if self.distance_to_wp < self.radius:
                    self.wp_number = self.wp_number + 1
                    break

            # -- check if the UAV reached the last waypoint within tolerance
            # -- if yes, end the waypoint traversal
            if self.distance_to_wp < self.radius and self.wp_number == self.num_wps:
                MissionFinished = True
                break

    # -- function for landing the UAV and closing the simulation
    def land(self):
        # -- return to initial launch site
        self.QRTL()
        
        # -- Close vehicle object before exiting script
        self.close_drone()

    # -- this function will get the distance between the UAV to the waypoint
    def get_distance_metres(self):
        """
        Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        # -- calculate the dlat and dlon of the current position to the waypoint
        self.dlat = float(self.wp[self.wp_number][8]) - self.vehicle.location.global_relative_frame.lat
        self.dlong = float(self.wp[self.wp_number][9]) - self.vehicle.location.global_relative_frame.lon
        self.norm = math.sqrt((self.dlat*self.dlat) + (self.dlong*self.dlong))
        self.distance_to_wp = self.norm * 1.113195e5

    def QRTL(self):
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("QRTL")

        # -- Before closing the vehicle, make sure that the drone landed
        while True:
            print(" Altitude: %s m" % self.vehicle.location.global_relative_frame.alt)

            time.sleep(0.25) # -- slow the printing of the altitude of the drone

            # -- Break the loop when the drone lands within some threshold.
            if self.vehicle.location.global_relative_frame.alt <= 0.15:
                print("UAV successfully landed back at launch")
                break

    # -- function to set the drone vehicle mode into loiter
    def loiter(self, loiter_time):
        print("Putting vehicle into loiter mode.")
        self.setMode("LOITER")
        self.vehicle.parameters['WP_LOITER_RAD'] = 120
        time.sleep(loiter_time) # -- wait 0.5s to make sure that vehicle is in loiter mode

    # -- function used to close out the drone safely
    def close_drone(self):
        self.vehicle.close()
        print("Mission completed")

    # -- this function will be used to change the mode of the drone
    def setMode(self, newMode):
        self.vehicle.mode = VehicleMode(newMode)
        self.mode_target = "VehicleMode: " + newMode
        self.mode_current = self.mode_target

    # -- this function will be used to get the current mode of the drone
    def getMode(self):
        self.mode_current = "VehicleMode: " + self.vehicle.mode.name