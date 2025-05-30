"""
Created on Tuesday May 31st, 2022

@author: Rafal Krzysiak and Derek Hollenbeck and Di An

This class will act as a communication between the
UAV (real or simulated) and dronekit/mission planner

Usage:
    ...
"""

# -- import required libraries
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from smbus import SMBus # i2c Bus package 
import time
import os.path
import csv
import math
#import obstacleAvoidance

# -- define a UAV class
class UAV:
    
    # -- default parameter constructor
    def __init__(self):
        self.oa_type = 1
        self.takeoff_alt = 20 
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
        self.OV_min_threhold = 200 # cm
        self.OV_done_flag = False
        self.OV_enable_flag = False
        self.kp = 0.08
        self.u_v1 = 0
        self.u_v2 = 0
        self.u_v3 = 0
        self.u_v4 = 0
        self.u_v_x = []
        self.u_v_y = []
        self.i2cbus = SMBus(1)
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
            print("Waypoint %s coordinates (lat, lon): %s, %s" % (self.wp_number, \
                                                        self.wp[self.wp_number][8], \
                                                        self.wp[self.wp_number][9]))

            # -- send the UAV the coordinates of the waypoint wanting to travel to
            if float(self.wp[self.wp_number][3]) == 16:
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
                    time.sleep(0.25)
    
                # -- check if the UAV reached the last waypoint within tolerance
                # -- if yes, end the waypoint traversal
                if self.distance_to_wp < self.radius and self.wp_number == 6:
                    MissionFinished = True
                    break
            else:
                self.wp_number = self.wp_number + 1

    # -- function for landing the UAV and closing the simulation
    def land(self):
        # -- return to initial launch site
        self.QRTL()       
        # -- Control input (rangefinder change from max 700cm)
        # -- Control output (lat and lon pos, alt doesnt change from current)    
        # -- turn on rangefinder listener
        # -- run obstacle avoidance

        # Turn on all rangefinders       

        if self.oa_type == 1:
            self.OV_done_flag, self.OV_enable_flag= self.obstacle_Avoidance_Rangefinders
            if self.OV_done_flag and not self.OV_enable_flag:
                print('Obstacle avoidance has been done successfully')
                # Implementing landing procedures
                # ---
                self.i2cbus.close()
            # -- turn off rangefinder listener
            else:
                print('Obstacle avoidance is not successful and we cannot land here')
                # Stop landing and do hovering immediately
                # ---
        else:
            if self.oa_type == 2:
                self.simpleObstacleAvoidance
            
        # -- Close vehicle object before exiting script
        self.close_drone()
        
    def rangefinder_callback(self):
        #attr_name not used here.      
        if self.last_rangefinder_distance == round(self.vehicle.rangefinder.distance, 1):
             return
        self.last_rangefinder_distance = round(self.vehicle.rangefinder.distance, 1)
        print("Rangefinder2 (metres): %s" % self.last_rangefinder_distance)
        return self.last_rangefinder_distance
        
    def rangefinders_update(self):
        
        self.i2cbus.write_byte(0x70,0x51)
        self.i2cbus.write_byte(0x6F,0x51)
        self.i2cbus.write_byte(0x6E,0x51)
        self.i2cbus.write_byte(0x6D,0x51)
        time.sleep(0.12)
        val1 = self.i2cbus.read_word_data(0x70,0xE1)
        val2 = self.i2cbus.read_word_data(0x6f,0xE1)
        val3 = self.i2cbus.read_word_data(0x6e,0xE1)
        val4 = self.i2cbus.read_word_data(0x6d,0xE1)

        self.rangefinder1 = (val1>>8) & 0xff | (val1 & 0xff)
        self.rangefinder2 = (val2>>8) & 0xff | (val2 & 0xff)
        self.rangefinder3 = (val3>>8) & 0xff | (val3 & 0xff)
        self.rangefinder4 = (val4>>8) & 0xff | (val4 & 0xff)

        self.rangefinders = [self.rangefinder1, self.rangefinder2, self.rangefinder3, self.rangefinder4]
        return self.rangefinders

    def detect_obstacles(self):
        if self.OV_min_threhold < self.rangefinders[0] < self.OV_distance_threshold or self.OV_min_threhold < self.rangefinders[1] < self.OV_distance_threshold or \
            self.OV_min_threhold < self.rangefinders[2] < self.OV_distance_threshold or self.OV_min_threhold < self.rangefinders[3] <= self.OV_distance_threshold:# distance in cm or m?
            self.OV_enable_flag = True
        else:
            self.OV_enable_flag = False
        return self.OV_enable_flag

    def p_controller(self, range_val):

        range_val = self.kp*(self.OV_distance_threshold - range_val)
    
        return range_val

    def obstacle_Avoidance_Rangefinders(self):  
        if (self.vehicle.mode.name == 'QRTL' or self.vehicle.mode.name == 'QLAND') and self.vehicle.groundspeed < 2:
            while self.OV_done_flag and self.OV_enable_flag:
                self.rangefinders = self.rangefinders_update(self)
                while self.detect_obstacles(self):

                    self.u_v1 = self.p_controller(self.rangefinders[0])
                    self.u_v2 = self.p_controller(self.rangefinders[1])
                    self.u_v3 = self.p_controller(self.rangefinders[2])
                    self.u_v4 = self.p_controller(self.rangefinders[3])
                    # （u_v_x, u_v_y） = (0,-u_v1) + (-u_v2,0) + (0, u_v3) + (u_v4, 0)
                    self.u_v_x = -self.u_v2 + self.u_v4 
                    self.u_v_y = -self.u_v1 + self.u_v3 

                    if self.u_v_x > 1: # 1 m/s
                        self.u_v_x = 1
                    elif self.u_v_y > 1:
                        self.u_v_x = 1

                    self.send_global_velocity(self.u_v_x,self.u_v_y) # unit is m/s
                    time.sleep(0.15)
                
                self.OV_done_flag = True
                self.OV_enable_flag = True
            else:
                print("Current Mode is not correct, please check back to land mode which is QRTL or QLAND")
        return self.OV_done_flag, self.OV_enable_flag  
     
    def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.

        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)    
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
            
    def get_curPosition(self):
        self.cur_lat = float(self.vehicle.location.global_relative_frame.lat)
        self.cur_lon = float( self.vehicle.location.global_relative_frame.lon)
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