#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def inchesToMeter(inches):
    return inches * 0.025


def metersToInches(meters):
    return meters / 0.0254


def radToDegree(rad):
    return rad * 180 / math.pi


# constants
minP = inchesToMeter(18)
maxP = inchesToMeter(300000)

# rover geometry
d1 = inchesToMeter(7.254)
d2 = inchesToMeter(10.5)
d3 = inchesToMeter(10.5)
d4 = inchesToMeter(10.073)

# input 
linVelocity = 0.0
angVelocity = 0.0

# Global variables
CmdVelocities = Twist()
prev_CmdVelocities = Twist()
WheelVelocities1 = Twist()
WheelVelocities2 = Twist()
WheelVelocities3 = Twist()
WheelVelocities4 = Twist()
WheelVelocities5 = Twist()
WheelVelocities6 = Twist()


# def extractVelocitiesFromTwistMessage(twistMessage):
#	return 0.0 #twistMessage.linVelocity

def calcP(angVelocity):
    return (maxP * math.exp(- abs(angVelocity) * 20) + minP) * np.sign(angVelocity)


# Velocoity Calculcation
def calcR1(r):
    return math.sqrt(d3 ** 2 + (d1 + r) ** 2)


def calcR2(r):
    return d4 + r


def calcR3(r):
    return math.sqrt(d2 ** 2 + (d1 + r) ** 2)


def calcR4(r):
    return math.sqrt(d3 ** 2 + (r - d1) ** 2)


def calcR5(r):
    return r - d4


def calcR6(r):
    return math.sqrt(d2 ** 2 + (r - d1) ** 2)


def calcV1(vel, r):
    return vel * calcR1(r) / calcR2(r)


def calcV2(vel, r):
    return vel


def calcV3(vel, r):
    return vel * calcR3(r) / calcR2(r)


def calcV4(vel, r):
    return vel * calcR4(r) / calcR2(r)


def calcV5(vel, r):
    return vel * calcR5(r) / calcR2(r)


def calcV6(vel, r):
    return vel * calcR6(r) / calcR2(r)


# steering angles - front wheels
def angleW1(r):
    return math.atan(d3 / (r + d1))


def angleW4(r):
    return math.atan(d3 / (r - d1))


# steering angles - back wheels
def angleW3(r):
    return -angleW1(r)


def angleW6(r):
    return -angleW4(r)


# mapping for left or right turn
def getWheel1(p, linVel):
    turnDirection = np.sign(p)
    r = abs(p)

    vel = linVel
    angle = 0.0

    if turnDirection != 0:  # we have to turn
        if turnDirection < 0:  # left
            vel = calcV4(linVel, r)
            angle = -angleW4(r)
        else:  # right
            vel = calcV1(linVel, r)
            angle = angleW1(r)

    d = dict()
    d["wheel"] = "Wheel 1"
    d["vel"] = vel
    d["angle"] = angle
    return d


def getWheel2(p, linVel):
    turnDirection = np.sign(p)
    r = abs(p)

    vel = linVel
    angle = 0.0

    if turnDirection != 0:  # we have to turn
        if turnDirection < 0:  # left
            vel = calcV5(linVel, r)
        else:  # right
            vel = calcV2(linVel, r)

    d = dict()
    d["wheel"] = "Wheel 2"
    d["vel"] = vel
    d["angle"] = angle
    return d


def getWheel3(p, linVel):
    turnDirection = np.sign(p)
    r = abs(p)

    vel = linVel
    angle = 0.0

    if turnDirection != 0:  # we have to turn
        if turnDirection < 0:  # left
            vel = calcV6(linVel, r)
            angle = -angleW6(r)
        else:  # right
            vel = calcV3(linVel, r)
            angle = angleW3(r)

    d = dict()
    d["wheel"] = "Wheel 3"
    d["vel"] = vel
    d["angle"] = angle
    return d


def getWheel4(p, linVel):
    turnDirection = np.sign(p)
    r = abs(p)

    vel = linVel
    angle = 0.0

    if turnDirection != 0:  # we have to turn
        if turnDirection < 0:  # left
            vel = calcV1(linVel, r)
            angle = -angleW1(r)
        else:  # right
            vel = calcV4(linVel, r)
            angle = angleW4(r)

    d = dict()
    d["wheel"] = "Wheel 4"
    d["vel"] = vel
    d["angle"] = angle
    return d


def getWheel5(p, linVel):
    turnDirection = np.sign(p)
    r = abs(p)

    vel = linVel
    angle = 0.0

    if turnDirection != 0:  # we have to turn
        if turnDirection < 0:  # left
            vel = calcV2(linVel, r)
        else:  # right
            vel = calcV5(linVel, r)

    d = dict()
    d["wheel"] = "Wheel 5"
    d["vel"] = vel
    d["angle"] = angle
    return d


def getWheel6(p, linVel):
    turnDirection = np.sign(p)
    r = abs(p)

    vel = linVel
    angle = 0.0

    if turnDirection != 0:  # we have to turn
        if turnDirection < 0:  # left
            vel = calcV3(linVel, r)
            angle = -angleW3(r)
        else:  # right
            vel = calcV6(linVel, r)
            angle = angleW6(r)

    d = dict()
    d["wheel"] = "Wheel 6"
    d["vel"] = vel
    d["angle"] = angle
    return d


def TwistCallback(cmdVel):
    CmdVelocities.linear.x = cmdVel.linear.x
    CmdVelocities.angular.z = cmdVel.angular.z
    print "lin x -------" + " " + str(CmdVelocities.linear.x)
    print "ang z -------" + " " + str(CmdVelocities.angular.z)


if __name__ == '__main__':

    sub = rospy.Subscriber('cmd_vel', Twist, TwistCallback)
    pub1 = rospy.Publisher('beetle_left_wheel_speed', Twist, queue_size=100)
    pub2 = rospy.Publisher('beetle_center_left_wheel_speed', Twist, queue_size=100)
    pub3 = rospy.Publisher('beetle_rear_left_wheel_speed', Twist, queue_size=100)
    pub4 = rospy.Publisher('beetle_right_wheel_speed', Twist, queue_size=100)
    pub5 = rospy.Publisher('beetle_center_right_wheel_speed', Twist, queue_size=100)
    pub6 = rospy.Publisher('beetle_rear_right_wheel_speed', Twist, queue_size=100)

    rospy.init_node('beetle_kinematic_model', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print "-----------------------"
        # while CmdVelocities != prev_CmdVelocities:
        angVel = CmdVelocities.angular.z
        linVel = CmdVelocities.linear.x
	print CmdVelocities

        w1 = getWheel1(calcP(angVel), linVel)
        w2 = getWheel2(calcP(angVel), linVel)
        w3 = getWheel3(calcP(angVel), linVel)
        w4 = getWheel4(calcP(angVel), linVel)
        w5 = getWheel5(calcP(angVel), linVel)
        w6 = getWheel6(calcP(angVel), linVel)

        WheelVelocities1.linear.x = w1["vel"]
        WheelVelocities1.angular.z = w1["angle"]
        WheelVelocities2.linear.x = w2["vel"]
        WheelVelocities2.angular.z = w2["angle"]
        WheelVelocities3.linear.x = w3["vel"]
        WheelVelocities3.angular.z = w3["angle"]
        WheelVelocities4.linear.x = w4["vel"]
        WheelVelocities4.angular.z = w4["angle"]
        WheelVelocities5.linear.x = w5["vel"]
        WheelVelocities5.angular.z = w5["angle"]
        WheelVelocities6.linear.x = w6["vel"]
        WheelVelocities6.angular.z = w6["angle"]

        pub1.publish(WheelVelocities1)
        pub2.publish(WheelVelocities2)
        pub3.publish(WheelVelocities3)
        pub4.publish(WheelVelocities4)
        pub5.publish(WheelVelocities5)
        pub6.publish(WheelVelocities6)
        rate.sleep()
    # prev_CmdVelocities = CmdVelocities
