# -*- coding: utf-8 -*-
"""
Created on Tue Jun  7 23:16:20 2022

@author: MESA LAB
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative

import time

import argparse

parser = argparse.ArgumentParser(description = 'commands')

parser.add_argument('–-connect')

args =parser.parse_args()

connection_string = args.connect

vehicle = connect(connection_string ,wait_ready=True)