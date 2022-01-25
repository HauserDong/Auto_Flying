#!/usr/bin/env python

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import t265_precland_apriltags
import os

#######################################
# Default Parameters Setting
#######################################

#Default configuration for connection to the FCU
connection_string_default = '127.0.0.1:14550'   #Change it according to the companion computer you're using


#######################################
# Parsing user's inputs
#######################################
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode.')
parser.add_argument('--connect', 
                    help = 'Vehicle connection target string. If not specified, a default string will be used.')
args = parser.parse_args()

connection_string = args.connect

#Using default values if no input is provided
if not connection_string:
    connection_string = connection_string_default
    print("INFO: Using default connection_string %s" %(connection_string))
else:
    print("INFO: Using connection_string %s" %(connection_string))


#######################################
# Connecting to Vehicle
#######################################
print("Connecting to vehicle on: %s" %(connection_string))
vehicle = connect(connection_string, wait_ready = True)


#######################################
# Functions for Dronekit
#######################################


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        print 'Altitude: ' , vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, #Setting frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)


#######################################
# Main code start here
#######################################

#First Running t265_precland_apriltags.py
command = 'python3 t265_precland_apriltags.py'
command_result = os.system(command)




arm_and_takeoff(1.2)    #Change it for your specific experiment, for instance, 1.2 meters in this case

print("Testing path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
DURATION = 20 #Set duration for each segment.

#Using AprilTag to determine the destination
move_forward = 25
move_right = -20
move_down = -5
print("Forward %dm, Right %dm, Down %dm  for %d seconds" % (move_forward, move_right, move_down, DURATION))
goto_position_target_local_ned(move_forward,move_right,move_down)
time.sleep(DURATION)

#Change to LAND mode
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

