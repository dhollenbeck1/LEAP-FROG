# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 19:31:02 2022

@author: MESA LAB
"""

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
vehicle = connect('tcp:127.0.0.1:5762',wait_ready=True)

# Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)    # settable

# Close vehicle object before exiting script
vehicle.close()
print("Completed")