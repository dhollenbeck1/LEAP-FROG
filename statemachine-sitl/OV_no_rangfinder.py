# -*- coding: utf-8 -*-
"""
Created on Sunday June 5th, 2022

@author: Di An

This script is for LEAP-FROG Obstacle avoidance between the
UAV (real or simulated) and dronekit/mission planner

Usage:
    ...
"""

# -- import required libraries
from dronekit import connect, VehicleMode, LocationGlobalRelative, Rangefinder
from pymavlink import mavutil # Needed for command message definitions
import VehicleClass2
import time
import os.path
import csv
import math

global mode_current, mode_target

# OV_enable_flag = False
# OV_done_flag = False
# vehicle_VTOL_mode = False
#OV_distance_threshold = 7 # in meter?
# P controller parameter for tunning
#kp = 0.08
LEAPFROG = VehicleClass2.UAV()

# def check_current_mode(mode_current):
#     if mode_current == 'QRTL':
#         LEAPFROG.vehicle_VTOL_mode = True
#     else: 
#         LEAPFROG.vehicle_VTOL_mode = False
#         print('Current UAV mode is not in VTOL, please check the vehicle mode')
#         # break /should return previous state
#     return LEAPFROG.vehicle_VTOL_mode

def detect_obstacles(LEAPFROG.ob_cur_distance):
    if LEAPFROG.rangefinder_voltage == True:# Not sure what are the normal voltage value and abnormal voltage value
        # listner
        if LEAPFROG.ob_cur_distance <= LEAPFROG.OV_distance_threshold:# distance in cm or m?
            LEAPFROG.OV_enable_flag = True
        else:
            LEAPFROG.OV_enable_flag = False
    else:
        print("Rangefinder Sensor Does Not Enabled, Please Check Rangefinder Sensor and Its Setup")
        # break /should return previous state
    return LEAPFROG.OV_enable_flag
            
# Using P controller to main hover
def main_workflow(mode_current):
    # scanLoop iteration parameter
    scanLoop = 1
    # Check VTOl Mode
    #if check_current_mode(mode_current):    
        # check OV_enable_flag
    while detect_obstacles(LEAPFROG.ob_cur_distance):
        # Altitude Hold 
        #cur_altitude = LEAPFROG.vehicle.location.global_relative_frame.alt
        # -- Not sure how to do altutide hold
        obstacles_distance = LEAPFROG.ob_cur_distance
        current_yaw = LEAPFROG.vehicle.attitude.yaw
        # Scan around from current heading position with offset 90 degrees at each time.
        # current heading
        while obstacles_distance is not LEAPFROG.OV_distance_threshold:
            u_v = p_controller(obstacles_distance)
            if scanLoop & 1 != 0:
                send_global_velocity(-u_v,0)
            else:
                send_global_velocity(0,-u_v)
            next_yaw = current_yaw + 90
            condition_yaw(next_yaw)
            # Get the current rangefinder distance, yaw
            obstacles_distance = LEAPFROG.ob_cur_distance
            current_yaw = LEAPFROG.vehicle.attitude.yaw
            scanLoop = scanLoop + 1
        LEAPFROG.ob_cur_distance = LEAPFROG.ob_cur_distance - 1

    
    LEAPFROG.OV_done_flag = True
    #else:
    #    LEAPFROG.OV_enable_flag = False
    return LEAPFROG.OV_enable_flag, LEAPFROG.OV_done_flag   

def p_controller(obstacles_distance):
    error = LEAPFROG.OV_distance_threshold - obstacles_distance
    u_velocity = LEAPFROG.kp*error
    return u_velocity    


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
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
    msg = LEAPFROG.vehicle.message_factory.set_position_target_global_int_encode(
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
        LEAPFROG.vehicle.send_mavlink(msg)
        time.sleep(1)    


def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = LEAPFROG.vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    LEAPFROG.vehicle.send_mavlink(msg)           
    
        
   
        
