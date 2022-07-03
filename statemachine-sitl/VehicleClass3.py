"""
Created on Tuesday May 31st, 2022

@author: Rafal Krzysiak and Derek Hollenbeck

This class will act as a communication between the
UAV (real or simulated) and dronekit/mission planner

Usage:
    ...
"""

# -- import required libraries
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
# from smbus import SMBus
import time
import os.path
import csv
import math
from pymavlink import mavutil
#import obstacleAvoidance

# -- define a UAV class
class UAV:
    
    # -- default parameter constructor
    def __init__(self):
        # self.i2cbus = SMBus(1)
        self.oa_type = 1
        self.takeoff_alt = 50 
        self.obs_speed = 1
        self.mode_current = "VehicleMode:Default"
        self.mode_target = "VehicleMode:GUIDED"
        self.last_rangefinder_distance=0
        self.cur_lat = 0
        self.cur_lon = 0
        self.cur_alt = 0
        self.obs_lat = 0
        self.obs_lon = 0
        self.obs_alt = 0
        self.threshold_dist = 0.0001
        self.gain_oa = 0.0001
        self.dlat = 0
        self.dlong = 0
        self.norm = 1
        self.num_wps = 0
        self.obs_dlat = 100
        self.obs_dlon = 100
        self.obs_dalt = 100
        self.control_lat = 0
        self.control_lon = 0
        self.control_alt = 0
        self.tar_lat = 0
        self.tar_lon = 0
        self.star_alt = 0
        self.OV_distance_threshold = 700 #cm
        self.OV_enable_flag = False
        self.OV_done_flag = False
        self.vehicle_VTOL_mode = False
        self.kp = 0.08
        # OV_no_rangfinder
        #cm, maximum, 700 means no obstacles.
        self.rng_Ts = 0.2
        self.rangefinder1 = 700
        self.rangefinder2 = 700
        self.rangefinder3 = 700
        self.rangefinder4 = 700
        self.rangefinders = [self.rangefinder1, self.rangefinder2, self.rangefinder3, self.rangefinder4]
        

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
                print("Waypoints defined: %s" % waypointsFile)
                #print(self.wp) # -- print out waypoints for debugging
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
    def connect_UAV(self, addrss, WAITREADY, flag):
        if flag == 1:   # -- for TCP
            self.vehicle = connect(addrss, wait_ready=WAITREADY)
        elif flag == 2:
            self.vehicle = connect(addrss, wait_ready=WAITREADY, baud=921600)
        elif flag == 3:           # -- for UDP
            self.vehicle = connect(addrss, wait_ready=WAITREADY, baud=115200)

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
        #self.vehicle.mode = VehicleMode("GUIDED")
        self.setMode("GUIDED")
        time.sleep(1)
        print("mode changed to: %s" % self.getMode())
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
        while True:
            if self.vehicle.location.global_relative_frame.alt < 1:
                self.vehicle.simple_takeoff(TargetAltitude)  # Take off to target altitude
            else:
                break 
            time.sleep(1)
            
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
    def Waypoint_Travel(self, airspeed):
        print("===========================")
        print("Beginning waypoint mission.")
        print("===========================")

        # -- set the UAV's airspeed and ground speed to the defined values
        self.vehicle.airspeed = airspeed
        # self.vehicle.groundspeed = groundspeed

        # -- initialize boolean for whether or not the UAV finished the waypoints
        MissionFinished = False

        # -- boolean value for transistioning to a new waypoint
        self.waypoint_met = False

        # -- begin the counter for waypoint number
        self.wp_number = 0
        self.radius = 90.0 # -- in meters
        self.vehicle.parameters['Q_WP_RADIUS'] = 10
        self.kr = 0.00025
        self.num_wps = len(self.wp)

        # -- begin looping through all waypoints
        # while True:
        for waypoint in range (1, self.num_wps):
            # -- print out the coordinates of the waypoint that the UAV is travelling to
            print("Waypoint %s coordinates (lat, lon): %s, %s" % (self.wp_number, \
                                                        self.wp[self.wp_number][8], \
                                                        self.wp[self.wp_number][9]))

            # -- in this instance of the function call, we are using the other values
            # -- self.dlat, self.dlong and self.norm to calcuate a virtual point for the 
            # -- drone to fly to so that the drone does not slow down in the air, This reduces battery waste
            self.get_distance_metres2()
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
            self.vehicle.simple_goto(point, self.vehicle.airspeed,None)

            # -- previous distance
            self.previous_dist = self.distance_to_wp
            
            # -- continue looping until UAV meets the waypoint
            while True:
                # -- get the distance to the waypoint
                self.previous_dist = self.distance_to_wp

                # -- used to slow down the printing of the distance from waypoint
                time.sleep(1)

                self.get_distance_metres2()
                self.virtual_wp = (float(self.wp[self.wp_number][8]) + (self.dlat + self.kr*(self.dlat/self.norm)),\
                                   float(self.wp[self.wp_number][9]) + (self.dlong + self.kr*(self.dlong/self.norm)))

                # -- send the UAV the coordinates of the waypoint wanting to travel to
                point = LocationGlobalRelative(self.virtual_wp[0], \
                                               self.virtual_wp[1], \
                                               float(self.wp[self.wp_number][10]))    

                self.vehicle.simple_goto(point, self.vehicle.airspeed,None)
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
        # -- Control input (rangefinder change from max 700cm)
        # -- Control output (lat and lon pos, alt doesnt change from current)    
        # -- turn on rangefinder listener
        # -- run obstacle avoidance
        
        if self.oa_type == 1:
            # OV_enable_flag, OV_done_flag = obstacleAvoidance.main_workflow(self.ob_distance_1)
            # if OV_done_flag and not OV_enable_flag:
            #     print('Obstacle avoidance has been done successfully')
            #     # Implementing landing procedures
            #     # ---
            # # -- turn off rangefinder listener
            # else:
            #     print('Obstacle avoidance is not successful and we cannot land here')
            #     # Stop landing and do hovering immediately
            #     # ---
            print('Disabled OA type 1')
        else:
            if self.oa_type == 2:
                self.simpleObstacleAvoidance
            
        # -- Close vehicle object before exiting script
        self.close_drone()
        

        
    # I don't recommend to use call_back function to monitoring the rangefinder sensor,
    # Since attributes which reflect vehicle “state” are only updated when their values change,
    # rangefinder sensor value is going to change rapidly which going to be chaos.
    def rangefinder_callback(self):
        #attr_name not used here.      
        if self.last_rangefinder_distance == round(self.vehicle.rangefinder.distance, 1):
             return
        self.last_rangefinder_distance = round(self.vehicle.rangefinder.distance, 1)
        print("Rangefinder2 (metres): %s" % self.last_rangefinder_distance)
        return self.last_rangefinder_distance

    #@vehicle.on_attribute('rangefinder1')
    # def rangefinder1_callback(self):
    #     #attr_name not used here.
    #     global last_rangefinder1_distance
    #     if last_rangefinder1_distance == round(self.vehicle.rangefinder.distance, 1):
    #          return
    #     last_rangefinder1_distance = round(self.vehicle.rangefinder.distance, 1)
    #     print("Rangefinder1 (metres): %s" % last_rangefinder1_distance)
        
        
    # def rangefinder2_callback(self):
    #     #attr_name not used here.
    #     global last_rangefinder2_distance
        
    #     if last_rangefinder2_distance == round(self.vehicle.rangefinder.distance, 1):
    #          return
    #     last_rangefinder2_distance = round(self.vehicle.rangefinder.distance, 1)
    #     print("Rangefinder2 (metres): %s" % last_rangefinder2_distance)


    # -- this function will get the distance between the UAV to the waypoint
    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.
        num_waypoints = 0
        for cmd in cmds:
            num_waypoints = num_waypoints + 1
            
        print('num of waypoints: %s' % num_waypoints)
        return num_waypoints
    
    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        #alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon)
        currentWaypointLocation = LocationGlobalRelative(self.vehicle.location.global_frame.lat, \
                                                         self.vehicle.location.global_frame.lon) 
        distancetopoint = self.get_distance_metres(currentWaypointLocation, targetWaypointLocation)
        return distancetopoint
      
    
    def get_distance_metres2(self):
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

    def get_obs_distance_metres(self):
        """
        Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """      
        # -- calculate the dlat and dlon of the current position to the waypoint
        self.obs_dlat = self.obs_lat - self.vehicle.location.global_relative_frame.lat
        self.obs_dlon = self.obs_lon - self.vehicle.location.global_relative_frame.lon
        self.obs_dalt = self.obs_alt - self.vehicle.location.global_relative_frame.alt
        return math.sqrt(((self.obs_dlat*self.obs_dlat) + \
                          (self.obs_dlon*self.obs_dlon)))
                         #self.obs_dalt*self.obs_dalt)1.113195e5
            
    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,original_location.alt)

    
    def get_curPosition(self):
        self.cur_lat = float(self.vehicle.location.global_relative_frame.lat)
        self.cur_lon = float(self.vehicle.location.global_relative_frame.lon)
        self.cur_alt = float(self.vehicle.location.global_relative_frame.alt)
        print("current position: %s, %s, %s" % (self.cur_lat,self.cur_lon,self.cur_alt))
        
    def simpleObstacleAvoidance(self):   
        #print("vehicle mode: %s" % self.vehicle.mode.name)
        
        # Check if we are in QRTL and vehicle speed is less than 2m/s
        if (self.vehicle.mode.name == 'QRTL' or self.vehicle.mode.name == 'QLAND') and self.vehicle.groundspeed < 2:
            cur_obs_dist = self.get_obs_distance_metres()
            print("distance to obstacle: %s" % cur_obs_dist)
            if cur_obs_dist < self.threshold_dist: 
                while True:
                    if cur_obs_dist < self.threshold_dist: 
                        self.setMode("GUIDED")
                        self.control_lat = -1 * self.gain_oa * (self.threshold_dist - self.obs_dlat)
                        self.control_lon = -1 * self.gain_oa * (self.threshold_dist - self.obs_dlon)
                        print("obstacle detected...control output: %s, %s" % (self.control_lat,self.control_lon))
                        cur_obs_dist = self.get_obs_distance_metres()
                        print("distance to obstacle: %s" % cur_obs_dist)
                        self.get_curPosition()
                        self.tar_lat = self.cur_lat + self.control_lat
                        self.tar_lon = self.cur_lon + self.control_lon
                        self.tar_alt = self.cur_alt
                        
                        
                        # -- Set mode to GUIDED and go to new control point before returning to QRTL()
                        point = LocationGlobalRelative(float(self.tar_lat), \
                                                       float(self.tar_lon), \
                                                    float(self.tar_alt))
                        self.vehicle.simple_goto(point, self.obs_speed)
                        
                        time.sleep(0.5)
                        #cur_obs_dist = self.get_obs_distance_metres()
                    else:
                        self.setMode("QLAND")
                        print("Object avoided")
                        break
            

    def QRTL(self):
        print("Returning to Launch")
        #self.vehicle.mode = VehicleMode("QRTL")
        self.setMode("QRTL")

        # -- Before closing the vehicle, make sure that the drone landed
        while True:
            print(" Altitude: %s m" % self.vehicle.location.global_relative_frame.alt)
            self.simpleObstacleAvoidance()
            time.sleep(1) # -- slow the printing of the altitude of the drone

            # -- Break the loop when the drone lands within some threshold.
            if self.vehicle.location.global_relative_frame.alt <= 0.15:
                print("UAV successfully landed back at launch")
                break
            
    def getMode(self):
        self.mode_current = "VehicleMode:" + self.vehicle.mode.name
        return self.mode_current
    
    def setMode(self, newMode):
        self.vehicle.mode = VehicleMode(newMode)
        self.mode_target = "VehicleMode:" + newMode
        self.mode_current = self.mode_target
        
    def getRngFinders(self):
        # while True:
        self.i2cbus.write_byte(0x70, 0x51)
        self.i2cbus.write_byte(0x6f, 0x51)
        time.sleep(self.rng_Ts)
        val1 = self.i2cbus.read_word_data(0x70, 0xE1)
        val2 = self.i2cbus.read_word_data(0x6f, 0xE1)
        rng1 = (val1 >> 8) & (0xff | (val1 & 0xff))
        rng2 = (val2 >> 8) & (0xff | (val2 & 0xff))
        rng3 = -1
        rng4 = -1
        print("rangfinder 1 %s cm" % rng1)
        print("rangfinder 2 %s cm" % rng2)
        self.rangfinders = [rng1,rng2,rng3,rng4]
        return self.rangfinders
        
    def compareMode(self, mode_current):
        #print("Mode target: %s, Mode current: %s" % (mode_target, mode_current))
        #print(str(mode_current)==mode_target)
        #print(type(mode_current))
        #print(type(mode_target))
        if (str(self.mode_current) == self.mode_target):
            print("Mode Change Valid")
            return 1
        else:
            print("Error: Mode Mismatch")
            return 0          
        
    def mode_callback(self, attr_name, msg, msg2):
        self.mode_current = msg2
        print("The current mode is: %s" % self.mode_current)
        self.compareMode(self.mode_current)
    
    #def removeListener_mode(self):           

    def close_drone(self):
        self.vehicle.close()
        print("Mission completed")
