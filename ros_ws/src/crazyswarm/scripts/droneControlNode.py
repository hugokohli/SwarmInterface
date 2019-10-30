#!/usr/bin/env python

import numpy as np
# ROS
import rospy
from std_msgs.msg import String, Duration, Header
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf.transformations

# Python
import time
from pynput import mouse
from math import sqrt
import threading

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg

from pycrazyswarm import *
import uav_trajectory


DRONE_RIGIDBODY = ["cf1"]
FLYING = 0
LANDED = 1
TAKINGOFF = 2
LANDING = 3

TAKE_OFF_HEIGHT = 0.5# m
TAKE_OFF_DURATION = 3.0 # s

LAND_HEIGHT = 0.06 # m
LAND_DURATION = 3.0 # s

COMMAND_SEND_RATE = 20 # Hz
COMMAND_UPDATE_RATE = 10 # Hz


MOCAP_INIT_TIMEOUT = 10 # s


droneState = LANDED
targetPositionInitialized = False


def takeoffandland():
    global droneState
    if (droneState==LANDED) :
        droneState = TAKINGOFF
        time.sleep(TAKE_OFF_DURATION)
        droneState = FLYING
        time.sleep(0.0)
        droneState = LANDING
        time.sleep(LAND_DURATION)
        droneState = LANDED

def updateState(takeoffCommand, landCommand):

    global droneState

    takeoffDuration = rospy.Duration.from_sec(TAKE_OFF_DURATION)
    landDuration = rospy.Duration.from_sec(LAND_DURATION)

    rate = rospy.Rate(COMMAND_SEND_RATE)
    while(droneState != LANDED):      
        if (droneState == TAKINGOFF) :
            takeoffCommand(groupMask=0, height=TAKE_OFF_HEIGHT, duration=takeoffDuration)
        elif (droneState == LANDING) :
            landCommand(groupMask=0, height=LAND_HEIGHT, duration=landDuration)
        rate.sleep()


def controlDrone():
# Create node
    rospy.init_node('droneControlNode')

    rospy.loginfo("Initializing drone control node.")
    
    takeoff = {}
    #goTo = {}
    land = {}

    for droneRigidBody in DRONE_RIGIDBODY :
        rospy.wait_for_service(droneRigidBody + '/takeoff')
        #rospy.wait_for_service(droneRigidBody + '/go_to')
        rospy.wait_for_service(droneRigidBody +'/land')

        # Acquire reference to the high-level command services
        takeoff[droneRigidBody] = rospy.ServiceProxy(droneRigidBody+'/takeoff', Takeoff)
        #goTo[droneRigidBody] = rospy.ServiceProxy(droneRigidBody+'/go_to', GoTo)
        land[droneRigidBody] = rospy.ServiceProxy(droneRigidBody+'/land', Land)
        rospy.loginfo(droneRigidBody)

    threading.Thread(target = takeoffandland).start()
    for droneRigidBody in DRONE_RIGIDBODY :
    	threading.Thread(target=updateState, args=(takeoff[droneRigidBody], land[droneRigidBody])).start()
    
    rate = rospy.Rate(COMMAND_UPDATE_RATE)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        controlDrone()
    except rospy.ROSInterruptException:
        pass