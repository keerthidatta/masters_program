#!/usr/bin/env python
# license removed for brevity
import math
from math import sin, cos, pi
import numpy as np
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


def inchesToMeter(inches):
    return inches * 0.0254


def metersToInches(meters):
    return meters / 0.0254


def radToDegree(rad):
    return rad * 180 / math.pi

def degreeToRad(degree):
    return degree * math.pi / 180


# From file Software Controls.pdf:
#   ... we therefore set the minimum radius to be at 20 inches to either side.
#   ... we say that anything above a turning radius of 250 inches is not turning at all and we give
#   all motors the same speed.
# TODO: So maybe set minP to 20 inches and maxP to 250 inches.

# constants
minP = inchesToMeter(18)
maxP = inchesToMeter(300000)

# rover geometry
d1 = inchesToMeter(7.254)
d2 = inchesToMeter(10.5)
d3 = inchesToMeter(10.5)
d4 = inchesToMeter(10.073)

# constants
maxRadius = inchesToMeter(10000000)
minAngle = math.atan(d3 / (maxRadius - d4))

#Declarations
WheelVelocities1 = Twist() # front left wheel
WheelVelocities2 = Twist() # center left wheel
WheelVelocities3 = Twist() # rear left wheel
WheelVelocities4 = Twist() # front right wheel
WheelVelocities5 = Twist() # center right wheel
WheelVelocities6 = Twist() # rear right wheel

WheelVelocities1.linear.x = 0
WheelVelocities1.angular.z = 0
WheelVelocities2.linear.x = 0
WheelVelocities2.angular.z = 0
WheelVelocities3.linear.x = 0
WheelVelocities3.angular.z = 0
WheelVelocities4.linear.x = 0
WheelVelocities4.angular.z = 0
WheelVelocities5.linear.x = 0
WheelVelocities5.angular.z = 0
WheelVelocities6.linear.x = 0
WheelVelocities6.angular.z = 0


# callback functions - wheel velocities
def frontleftwheelCallback(frontleftwheelPtr):

    if not math.isnan(frontleftwheelPtr.linear.x):
        WheelVelocities1.linear.x = frontleftwheelPtr.linear.x

    if not math.isnan(frontleftwheelPtr.angular.z):
        WheelVelocities1.angular.z = frontleftwheelPtr.angular.z


def centerleftwheelCallback(centerleftwheelPtr):

    if not math.isnan(centerleftwheelPtr.linear.x):
        WheelVelocities2.linear.x = centerleftwheelPtr.linear.x

    if not math.isnan(centerleftwheelPtr.angular.z):
        WheelVelocities2.angular.z = centerleftwheelPtr.angular.z


def rearleftwheelCallback(rearleftwheelPtr):

    if not math.isnan(rearleftwheelPtr.linear.x):
        WheelVelocities3.linear.x = rearleftwheelPtr.linear.x

    if not math.isnan(rearleftwheelPtr.angular.z):
        WheelVelocities3.angular.z = rearleftwheelPtr.angular.z


def frontrightwheelCallback(frontrightwheelPtr):

    if not math.isnan(frontrightwheelPtr.linear.x):
        WheelVelocities4.linear.x = frontrightwheelPtr.linear.x

    if not math.isnan(frontrightwheelPtr.angular.z):
        WheelVelocities4.angular.z = frontrightwheelPtr.angular.z


def centerrightwheelCallback(centerrightwheelPtr):

    if not math.isnan(centerrightwheelPtr.linear.x):
        WheelVelocities5.linear.x = centerrightwheelPtr.linear.x

    if not math.isnan(centerrightwheelPtr.angular.z):
        WheelVelocities5.angular.z = centerrightwheelPtr.angular.z


def rearrightwheelCallback(rearrightwheelPtr):

    if not math.isnan(rearrightwheelPtr.linear.x):
        WheelVelocities6.linear.x = rearrightwheelPtr.linear.x

    if not math.isnan(rearrightwheelPtr.angular.z):
        WheelVelocities6.angular.z = rearrightwheelPtr.angular.z



def getCurrentWheelSpeeds():

    V1 = WheelVelocities1.linear.x
    V2 = WheelVelocities2.linear.x
    V3 = WheelVelocities3.linear.x
    V4 = WheelVelocities4.linear.x
    V5 = WheelVelocities5.linear.x
    V6 = WheelVelocities6.linear.x

    return V1, V2, V3, V4, V5, V6


def getCurrentWheelAngles():

    theta1 = WheelVelocities1.angular.z
    theta2 = WheelVelocities2.angular.z
    theta3 = WheelVelocities3.angular.z
    theta4 = WheelVelocities4.angular.z
    theta5 = WheelVelocities5.angular.z
    theta6 = WheelVelocities6.angular.z

    return theta1, theta2, theta3, theta4, theta5, theta6


# Calculate Distance from Point P to each wheel
def calcR1(r1):
    return math.sqrt(d3 ** 2 + (d1 + r1) ** 2)


def calcR2(r2):
    return d4 + r2


def calcR3(r3):
    return math.sqrt(d2 ** 2 + (d1 + r3) ** 2)


def calcR4(r4):
    return math.sqrt(d3 ** 2 + (r4 - d1) ** 2)


def calcR5(r5):
    return r5 - d4


def calcR6(r6):
    return math.sqrt(d2 ** 2 + (r6 - d1) ** 2)


def calculateR(r1, r2, r3, r4, r5, r6):

    R1 = calcR1(r1)
    R2 = calcR2(r2)
    R3 = calcR3(r3)
    R4 = calcR4(r4)
    R5 = calcR5(r5)
    R6 = calcR6(r6)

    return R1, R2, R3, R4, R5, R6



# Calculate distance from Point P (which is different for every steerable wheel) to the center of the rover
def calcr1(theta1):
    assert theta1 != 0
    return (d3 - math.tan(theta1) * d1) / math.tan(theta1)


def calcr3(theta3):
    assert theta3 != 0
    return (-d3 - math.tan(theta3) * d1) / math.tan(theta3)


def calcr4(theta4):
    assert theta4 != 0
    return d1 + d3 / math.tan(theta4)


def calcr6(theta6):
    assert theta6 != 0
    return d1 - d3 / math.tan(theta6)


def calculate_rSteerableWheels(theta1, theta3, theta4, theta6):

    # if angle is minimal, set radius to maximal value. wheel is basically going straight ahead.
    if abs(theta1) < minAngle:
        r1 = maxRadius + 1
    else:
        r1 = calcr1(theta1)

    if abs(theta3) < minAngle:
        r3 = maxRadius + 1
    else:
        r3 = calcr3(theta3)

    if abs(theta4) < minAngle:
        r4 = maxRadius + 1
    else:
        r4 = calcr4(theta4)

    if abs(theta6) < minAngle:
        r6 = maxRadius + 1
    else:
        r6 = calcr6(theta6)

    #print r1
    #print r3
    #print r4
    #print r6

    return r1, r3, r4, r6


def calculateRoverSpeed(V1, V2, V3, V4, V5, V6, r1, r2, r3, r4, r5, r6):

    R1, R2, R3, R4, R5, R6 = calculateR(r1, r2, r3, r4, r4, r6)
    # center_speed_# = speed of rover center calculated out of the speed and radius of each individual wheel
    center_speed_1 = V1 * r1 / R1
    center_speed_2 = V2 * r2 / R2
    center_speed_3 = V3 * r3 / R3
    center_speed_4 = V4 * r4 / R4
    center_speed_5 = V5 * r5 / R5
    center_speed_6 = V6 * r6 / R6

    # If radius was set to maxRadius in function calculate_rSteerableWheels, the wheels basically go straight ahead.
    # So center speed is equal to wheel speed
    if abs(r1) >= maxRadius:
        center_speed_1 = V1

    if abs(r3) >= maxRadius:
        center_speed_3 = V3

    if abs(r4) >= maxRadius:
        center_speed_4 = V4

    if abs(r6) >= maxRadius:
        center_speed_6 = V6

    speed = np.mean([center_speed_1, center_speed_2, center_speed_3, center_speed_4, center_speed_5, center_speed_6])

    return speed

''' change V1... V6 and theta1 ... theta6 instead of wheel velocities.
def setTestValues():
    WheelVelocities1.linear.x = 1
    WheelVelocities1.angular.z = -math.pi / 4
    WheelVelocities2.linear.x = 1
    WheelVelocities2.angular.z = 0
    WheelVelocities3.linear.x = 1
    WheelVelocities3.angular.z = math.pi / 4
    WheelVelocities4.linear.x = 1
    WheelVelocities4.angular.z = -math.pi / 4
    WheelVelocities5.linear.x = 1
    WheelVelocities5.angular.z = 0
    WheelVelocities6.linear.x = 1
    WheelVelocities6.angular.z = math.pi / 4
'''


def setTestSpeeds():

    test_speed = 0.1
    V1 = test_speed
    V2 = test_speed
    V3 = test_speed
    V4 = test_speed
    V5 = test_speed
    V6 = test_speed

    return V1, V2, V3, V4, V5, V6

def setTestAngles():

    test_angle = 0
    theta1 = test_angle
    theta2 = test_angle
    theta3 = test_angle
    theta4 = test_angle
    theta5 = test_angle
    theta6 = test_angle

    return theta1, theta2, theta3, theta4, theta5, theta6


def calculateBeetleSpeedAndRadius():

    #setTestValues()

    V1, V2, V3, V4, V5, V6 = getCurrentWheelSpeeds()

    theta1, theta2, theta3, theta4, theta5, theta6 = getCurrentWheelAngles()

    ###################### TEST ######################

    #V1, V2, V3, V4, V5, V6 = setTestSpeeds()
    #theta1, theta2, theta3, theta4, theta5, theta6 = setTestAngles()

    ###################### TEST ######################

    r1, r3, r4, r6 = calculate_rSteerableWheels(theta1, theta3, theta4, theta6)

    # First approach of calculating the radius of the whole rover. This may lead to an unwanted small radius if
    # two of the radii are very large in the positive direction and two of the radii are very large in the negative
    # direction.
    # mean_radius = np.mean([r1, r3, r4, r6])
    # r2 = mean_radius
    # r5 = mean_radius
    # radius = mean_radius

    # Second approach of calculating the radius of the whole rover
    # positive theta1 and theta4 lead to a left turn as well as negative theta3 and theta6
    mean_angle = np.mean([theta1, -theta3, theta4, -theta6])

    if abs(theta1) < minAngle:
        rover_radius = maxRadius + 1
    else:
        assert mean_angle != 0
        rover_radius = d3 / math.tan(mean_angle)

    r2 = rover_radius
    r5 = rover_radius
    radius = rover_radius

    speed = calculateRoverSpeed(V1, V2, V3, V4, V5, V6, r1, r2, r3, r4, r5, r6)


    return speed, radius


def calculatePositionChange(dt, theta):

    speed, radius = calculateBeetleSpeedAndRadius()

    # Testvalues
    # speed = 1
    # radius = -2
    # dt = degreeToRad(45)

    distance = speed * dt

    delta_theta = distance / abs(radius)


    if (abs(radius) >= maxRadius):
        # move straight ahead
        dx = distance
        dy = 0
    else:
        # calculate movement along arc of a circle
        dx = sin(delta_theta) * radius
        dy = (cos(delta_theta) - 1) * (-radius)

    # rotate according to previous angle
    delta_x = (dx * cos(-theta)) + (dy * sin(-theta))
    delta_y = -(dx * sin(-theta)) + (dy * cos(-theta))



    # traslation = np.sqrt(np.power(prev_x, x, 2) +  np.power(prev_y, y, 2))
    # x = traslation * cos(theta)
    # y = traslation * sin(theta)
    # theta = math.atan(y / x)
    #
    # prev_x = x
    # prev_y = y
    # prev_theta = theta

    return delta_x, delta_y, delta_theta





if __name__ == '__main__':


    sub1 = rospy.Subscriber('beetle_front_left_wheel_converted', Twist, frontleftwheelCallback)
    sub2 = rospy.Subscriber('beetle_center_left_wheel_converted', Twist, centerleftwheelCallback)
    sub3 = rospy.Subscriber('beetle_rear_left_wheel_converted', Twist, rearleftwheelCallback)
    sub4 = rospy.Subscriber('beetle_front_right_wheel_converted', Twist, frontrightwheelCallback)
    sub5 = rospy.Subscriber('beetle_center_right_wheel_converted', Twist, centerrightwheelCallback)
    sub6 = rospy.Subscriber('beetle_rear_right_wheel_converted', Twist, rearrightwheelCallback)

    #sub1 = rospy.Subscriber('beetle_left_wheel_speed', Twist, frontleftwheelCallback)
    #sub2 = rospy.Subscriber('beetle_center_left_wheel_speed', Twist, centerleftwheelCallback)
    #sub3 = rospy.Subscriber('beetle_rear_left_wheel_speed', Twist, rearleftwheelCallback)
    #sub4 = rospy.Subscriber('beetle_right_wheel_speed', Twist, frontrightwheelCallback)
    #sub5 = rospy.Subscriber('beetle_center_right_wheel_speed', Twist, centerrightwheelCallback)
    #sub6 = rospy.Subscriber('beetle_rear_right_wheel_speed', Twist, rearrightwheelCallback)

    rospy.init_node('beetle_wheel_odometry', anonymous=True)


    # Source for code below: https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

    odom_pub = rospy.Publisher("beetle_wheel_odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    theta = 0.0


    # vx = 0.1
    # vy = -0.1
    # vth = 0.1

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(1000)
    #r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print "running"
        current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()

        # delta_x = (vx * cos(th) - vy * sin(th)) * dt
        # delta_y = (vx * sin(th) + vy * cos(th)) * dt
        # delta_th = vth * dt

        delta_x, delta_y, delta_theta = calculatePositionChange(dt, theta)

        x += delta_x
        y += delta_y
        theta += delta_theta

        print x
        print y
        print theta

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "beetle__base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        for i in range(0, 6):
            odom.pose.covariance[i * 7] = 0.01

        # set the velocity
        odom.child_frame_id = "beetle__base_link"
        #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()
