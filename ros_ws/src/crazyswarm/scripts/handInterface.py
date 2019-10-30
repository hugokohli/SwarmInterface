#!/usr/bin/env python

################################################################################
# Modules
################################################################################

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

################################################################################
# Constants
################################################################################

# Rigidbody names set in Optitrack Motive
HAND_RIGIDBODY = "hand"
DRONE_RIGIDBODY = "cf1"

MOCAP_INIT_TIMEOUT = 10 # s

COMMAND_UPDATE_RATE = 10 # Hz
COMMAND_SEND_RATE = 20 # Hz

# States of the drone
LANDED = 0
TAKING_OFF = 1
LANDING = 2
FLYING = 3

TAKE_OFF_HEIGHT = 0.2 # m
TAKE_OFF_DURATION = 1.0 # s

LAND_HEIGHT = 0.1 # m
LAND_DURATION = 3.0 # s

# Scaling of hand displacement to drone displacement
HAND_ROOM_SCALING = 2.0

# Rotation speed factor applied to wrist yaw
# TODO: Rotation not implemented yet
ROTATION_SPEED_SCALING = 0.02

# Increase speed of drone for more aggressive motion
DRONE_SPEED_MULTIPLIER = 1.0

################################################################################
# Global variables
################################################################################

clutchActivated = False
# if set to true only when the click is pressed, then set back to false immidiately
clutchTriggered = False

# mocapInputRotation = 0 --> you are facing the wall, and have the computers to your left and the entrance door to your right.
# mocapInputRotation = -90 --> you are facing the computers
# mocapInputRotation = 90 --> you have the computers behind you
mocapInputRotation = 0.0
observationInputRotation = 0.0 # computers behind you

rawHandPosition = Point(0.0, 0.0, 0.0)
oldRawHandPosition = Point(0.0, 0.0, 0.0)
deltaHandPosition = Point(0.0, 0.0, 0.0)

rawHandRotation = Quaternion()
handYaw = 0.0
referenceYaw = 0.0

# Target sent to the drone
handTarget = Point(0.0, 0.0, 0.0)
targetPositionInitialized = False

# Position of the drone
dronePosition = Point(0.0, 0.0, 0.0)

droneState = LANDED

################################################################################
# Utility functions
################################################################################

def getYaw (quat):
    """ quat in Quaternion message format """
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
    return yaw

def rotatePoint(quat, point):
    """ quat in Quaternion message format, point in Point message format """
    q1 = [quat.x, quat.y, quat.z, quat.w]

    v1 = [point.x, point.y, point.z]
    q2 = list(v1)
    q2.append(0.0)
    vr = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

    return Point(vr[0], vr[1], vr[2])

# Point message type does not provide arithmetic operators ##########3
def substractPoints(p1, p2):
    p3 = Point()

    p3.x = p1.x - p2.x
    p3.y = p1.y - p2.y
    p3.z = p1.z - p2.z

    return p3

def addPoints(p1, p2):
    p3 = Point()

    p3.x = p1.x + p2.x
    p3.y = p1.y + p2.y
    p3.z = p1.z + p2.z

    return p3

def scalePoint(p, val):
    s = Point(p.x, p.y, p.z)

    s.x = s.x * val
    s.y = s.y * val
    s.z = s.z * val

    return s

def updateHeader(header):
    """ Updates the ROS header, with the current time and increase its seq id"""
    header.seq = header.seq + 1
    header.stamp=rospy.Time.now()

################################################################################
# Core functions
################################################################################

def sendPositionCommandThread(goToCommand, takeoffCommand, landCommand):
    """ Thread sending the position target"""
    global handTarget
    global droneState

    gotoDuration = rospy.Duration.from_sec(1.0 / COMMAND_UPDATE_RATE / DRONE_SPEED_MULTIPLIER)
    takeoffDuration = rospy.Duration.from_sec(TAKE_OFF_DURATION)
    landDuration = rospy.Duration.from_sec(LAND_DURATION)

    rate = rospy.Rate(COMMAND_SEND_RATE)
    while not rospy.is_shutdown():
        if droneState == FLYING:
            #rospy.loginfo("Command: {0}".format(handTarget))
            goToCommand(groupMask=0, relative=False, goal=handTarget, yaw=0.0, duration=gotoDuration)
        elif droneState == TAKING_OFF:
            takeoffCommand(groupMask=0, height=TAKE_OFF_HEIGHT, duration=takeoffDuration)
        elif droneState == LANDING:
            landCommand(groupMask=0, height=LAND_HEIGHT, duration=landDuration)
            pass
        rate.sleep()

    return

def waitForTakeOff(duration):
    """ Update the drone state to flying after a certain duration """
    global droneState

    time.sleep(duration)
    droneState = FLYING

    return

def waitForLanding(duration):
    """ Update the drone state to landed after a certain duration """
    global droneState

    time.sleep(duration)
    droneState = LANDED

    return

# mocapPos is of type PoseStamped
def updateHandPosition(mocapPos):
    """ Callback called when a new position of the hand is sent by the mocap.
        Updates the global variables holding the pose of the hand. """
    global rawHandPosition
    global rawHandRotation

    rawHandPosition = mocapPos.pose.position
    rawHandRotation = mocapPos.pose.orientation

# Get drone position from mocap to initialized the target at the drone position
def updateDronePosition(mocapPos):
    """ Callback called when a new position of the drone is sent by the mocap.
        Updates the global variable holding the position of the drone.
        It will initialize the target position at startup with the current
        position of the drone."""
    global targetPositionInitialized
    global handTarget
    global dronePosition

    # Initialise target position at the current location of the drone
    if not targetPositionInitialized:
        handTarget = mocapPos.pose.position
        targetPositionInitialized = True
        rospy.loginfo("Initialized target position")

    dronePosition = mocapPos.pose.position

def initHandTracking(handRigidbodyName, droneRigidbodyName):
    """ Inits subscribers to rigidbodies tracked by the mocap system.
        We are interested in the position of the hand and the drone. """
    global targetPositionInitialized

    rospy.Subscriber("vrpn_client_node/" + droneRigidbodyName + "/pose", PoseStamped, updateDronePosition)

    # Wait until target position initialized, which is done in the updateDronePosition() callback
    sleepTime = 0.1
    elapsed = 0.0
    while not targetPositionInitialized and elapsed < MOCAP_INIT_TIMEOUT:
        time.sleep(sleepTime)
        elapsed = elapsed + sleepTime

    if not targetPositionInitialized:
        rospy.logwarn("Target position not initialized (Mocap timeout)")

    rospy.Subscriber("vrpn_client_node/" + handRigidbodyName + "/pose", PoseStamped, updateHandPosition)

    mouseListener = mouse.Listener(on_click=onClickCallback)
    mouseListener.start()

def onClickCallback(x, y, button, pressed):
    """ Callback for the mouse clicks, which activates/deactivates the clutch
        and triggers take-off and landing """
    global clutchActivated
    global clutchTriggered
    global referenceYaw
    global handYaw
    global droneState

    if button ==  mouse.Button.left:
        if (pressed):
            clutchActivated = True
            clutchTriggered = True
            referenceYaw = handYaw;
        else:
            clutchActivated = False
    elif button ==  mouse.Button.right:
        if (pressed):
            if droneState == LANDED:
                droneState = TAKING_OFF
                threading.Thread(target=waitForTakeOff, args=(TAKE_OFF_DURATION,)).start()
            elif droneState == FLYING:
                droneState = LANDING
                threading.Thread(target=waitForLanding, args=(TAKE_OFF_DURATION,)).start()

def clampInSafeArea(target):
    """ Limit the target position to a defined space, to avoid collisions
    with the floor, walls and ceiling."""
    if target.x > 3:
        target.x = 3
    if target.x < -3:
        target.x = -3
    if target.y > 3:
        target.y = 3
    if target.y < -3:
        target.y = -3
    if target.z > 3:
        target.z = 3
    if target.z < TAKE_OFF_HEIGHT:
        target.z = TAKE_OFF_HEIGHT

    return target


# Low-level commands cannot be mixed with high-level commands
#def lowLevelCommands(x,y,z):
#    """ Sends a low-level command for a set point."""
#    posCommandTopic = rospy.Publisher('cf1/cmd_position', PositionMsg, queue_size=10)
#    header = Header(seq=1, stamp=rospy.Time.now(), frame_id='world')
#    posCommandTopic.publish(PositionMsg(header=header, x=x, y=y, z=z, yaw=0.0))


def controlDrone():
    global rawHandPosition
    global oldRawHandPosition
    global deltaHandPosition
    global handTarget
    global clutchActivated
    global clutchTriggered
    global currentState
    global droneState

    # Create node
    rospy.init_node('handInterface')

    rospy.loginfo("Initializing hand interface node.")

    initHandTracking(HAND_RIGIDBODY, DRONE_RIGIDBODY)

    # Wait for necessary service: drone commands
    rospy.wait_for_service(DRONE_RIGIDBODY + '/takeoff')
    rospy.wait_for_service(DRONE_RIGIDBODY + '/go_to')
    rospy.wait_for_service(DRONE_RIGIDBODY +'/land')

    # Acquire reference to the high-level command services
    takeoff = rospy.ServiceProxy(DRONE_RIGIDBODY+'/takeoff', Takeoff)
    goTo = rospy.ServiceProxy(DRONE_RIGIDBODY+'/go_to', GoTo)
    land = rospy.ServiceProxy(DRONE_RIGIDBODY+'/land', Land)

    droneState = LANDED

    # Start thread sending the position commands
    t = threading.Thread(target=sendPositionCommandThread, args=(goTo, takeoff, land))
    t.start()

    rospy.loginfo("Initialized successfully.")

    header = Header(seq=1, stamp=rospy.Time.now(), frame_id='world')

    # ROS loop
    rate = rospy.Rate(COMMAND_UPDATE_RATE)
    while not rospy.is_shutdown():

        deltaHandPosition = substractPoints(rawHandPosition, oldRawHandPosition)
        handYaw = getYaw(rawHandRotation)

        oldRawHandPosition = rawHandPosition

        if droneState == FLYING:
            if clutchActivated == True:
                if clutchTriggered == True:
                    clutchTriggered = False
                    handTarget = dronePosition
                #rospy.loginfo("Clutch")
                #droneVelocityControl.desiredYawRate = Mathf.DeltaAngle(referenceYaw, handYaw) * ROTATION_SPEED_SCALING
            else:
                #droneVelocityControl.desiredYawRate = 0.0

                # Ignore large deltas (dangerous)
                if (sqrt(deltaHandPosition.x * deltaHandPosition.x + deltaHandPosition.y * deltaHandPosition.y + deltaHandPosition.z * deltaHandPosition.z) > 1.0):
                    continue

                rot = tf.transformations.quaternion_from_euler (0, observationInputRotation + mocapInputRotation, 0)

                # Convert from list to Message type
                directionRotation = Quaternion(rot[0], rot[1], rot[2], rot[3])

                handTarget = addPoints(handTarget, scalePoint(rotatePoint(directionRotation, deltaHandPosition), HAND_ROOM_SCALING))
                handTarget = clampInSafeArea(handTarget)
        else:
            handTarget = dronePosition
            pass

        rate.sleep()


if __name__ == '__main__':
    try:
        controlDrone()
    except rospy.ROSInterruptException:
        pass
