#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
import roboclaw_driver as roboclaw
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
#from threading import Lock
from enum import Enum
import threading


#Declarations
leftwheel = Twist()
centerleftwheel = Twist() 
rearleftwheel = Twist() 
rightwheel = Twist() 
centerrightwheel = Twist() 
rearrightwheel = Twist() 

#callback functions - wheel velocities
def leftwheelCallback(leftwheelPtr):
	leftwheel.linear.x = leftwheelPtr.linear.x
	leftwheel.angular.z = leftwheelPtr.angular.z
	#print "leftwheel " + " "+ str(leftwheel.linear.x) + " " + str(leftwheel.angular.z)

def centerleftwheelCallback(centerleftwheelPtr):
	centerleftwheel.linear.x = centerleftwheelPtr.linear.x
	centerleftwheel.angular.z = centerleftwheelPtr.angular.z

def rearleftwheelCallback(rearleftwheelPtr):
	rearleftwheel.linear.x = rearleftwheelPtr.linear.x
	rearleftwheel.angular.z = rearleftwheelPtr.angular.z

def rightwheelCallback(rightwheelPtr):
	rightwheel.linear.x = rightwheelPtr.linear.x
	rightwheel.angular.z = rightwheelPtr.angular.z

def centerrightwheelCallback(centerrightwheelPtr):
	centerrightwheel.linear.x = centerrightwheelPtr.linear.x
	centerrightwheel.angular.z = centerrightwheelPtr.angular.z

def rearrightwheelCallback(rearrightwheelPtr):
	rearrightwheel.linear.x = rearrightwheelPtr.linear.x
	rearrightwheel.angular.z = rearrightwheelPtr.angular.z

#roboclaw_mutex = Lock()
def AccessRoboclaw():
	#roboclaw_mutex.acquire()
	roboclaw.Open("/dev/beetle_roboclaws", 38400)
#def LeaveRoboclaw():
	#roboclaw_mutex.release()
	

# concepts for the conversion functions
class MotorSelect:
	WHEEL_FRONT_LEFT = [129, 1]
	WHEEL_FRONT_RIGHT = [129, 2]
	WHEEL_CENTER_LEFT = [128, 2]
	WHEEL_CENTER_RIGHT = [130, 1]
	WHEEL_BACK_LEFT = [128, 1]
	WHEEL_BACK_RIGHT = [130, 2]
	STEERING_FRONT_LEFT = [132, 1]
	STEERING_FRONT_RIGHT = [131, 2]
	STEERING_BACK_LEFT = [132, 2]
	STEERING_BACK_RIGHT = [131, 1]
	CAM_PAN = [133, 1]
	CAM_TILT = [133, 2]

def SetSpeedRover(motor_sel, enc_speed):
	AccessRoboclaw()
	if motor_sel[1] == 1:
		roboclaw.SpeedM1(motor_sel[0], int(enc_speed))
	elif motor_sel[1] == 2:
		roboclaw.SpeedM2(motor_sel[0], int(enc_speed))
	else:
		raise NotImplementedError

def GetEncRover(motor_sel):
	AccessRoboclaw()
	if motor_sel[1] == 1:
		enc_val = list(roboclaw.ReadEncM1(motor_sel[0]))
	elif motor_sel[1] == 2:
		enc_val = list(roboclaw.ReadEncM2(motor_sel[0]))
	else:
		raise NotImplementedError

	#if enc_val[0] == 0:
	#	"Could not read encoder value for " + str(motor_sel)
	#if enc_val[0] == 0 or enc_val[2] != motor_sel[0]: # received signal from wrong encoder, so we invalidate the result
	#	enc_val[0] = 0
	return enc_val

def GetEncSpeed(motor_sel):
	AccessRoboclaw()
	if motor_sel[1] == 1:
		enc_val = list(roboclaw.ReadSpeedM1(motor_sel[0]))
	elif motor_sel[1] == 2:
		enc_val = list(roboclaw.ReadSpeedM2(motor_sel[0]))
	else:
		raise NotImplementedError

	#if enc_val[0] == 0:
	#	"Could not read encoder value for " + str(motor_sel)
	#if enc_val[0] == 0 or enc_val[2] != motor_sel[0]: # received signal from wrong encoder, so we invalidate the result
	#	enc_val[0] = 0
	return enc_val


def SetPosBlocking(motor_sel, enc_speed, position): # maybe use setpositionPID instead?
	while True:
		current_pos = GetEncRover(motor_sel)
		if current_pos[0] == 1: # means we have a valid value from the encoder
			break
	print "initial pos: " + str(current_pos)
	position = int(position)
	if position < current_pos[1]:
		enc_speed = enc_speed * (-1)
	print "target position: " + str(position)
	print "velocity: " + str(enc_speed)
	SetSpeedRover(motor_sel, enc_speed)
	while True:
		current_pos = GetEncRover(motor_sel)
		if current_pos[0] == 1:
			print "intermediate position: " + str(current_pos)
			if enc_speed > 0 and current_pos[1] >= position:
				break

			if enc_speed < 0 and current_pos[1] <= position:
				break
	SetSpeedRover(motor_sel, 0)

def SetPosPID(motor_sel, position):
	AccessRoboclaw()
	speed, accel = 1000,2000 # maybe fine tune these
	if motor_sel[1] == 1:
		# SetM1PositionPID(motor_sel[0], kp, ki, kd, kimax, deadzone, limits[0], limits[1])
		roboclaw.SpeedAccelDeccelPositionM1(motor_sel[0], accel, speed, accel, position, 1)
	elif motor_sel[1] == 2:
		roboclaw.SpeedAccelDeccelPositionM2(motor_sel[0], accel, speed, accel, position, 1)
	else:
		raise NotImplementedError


wheel_diameter = 0.15
ticks_per_rotation = 18576
steering_angle_range = 3.1415926535897932 / 2
steering_enc_range = 171.79 * 1 / 2048 / 4 * (48)

safety_constant = 25
class SteeringLimits: # should be the encoder values at the limits. if they change with resets, we need to calibrate them first. shouldn't be a biggie
	#STEERING_FRONT_LEFT = [133, 1307]
	STEERING_FRONT_LEFT = [150, 1300]
	STEERING_FRONT_RIGHT = [280, 1423]
	STEERING_BACK_LEFT = [259, 1144] #TODO measure those constants
	STEERING_BACK_RIGHT = [210, 1364] #TODO measure those constants
	CAM_TILT = [638, 882] #TODO measure those constants


def MapMotorSelToLimits(motor_sel):
	if motor_sel == MotorSelect.STEERING_FRONT_LEFT:
		return SteeringLimits.STEERING_FRONT_LEFT
	if motor_sel == MotorSelect.STEERING_FRONT_RIGHT:
		return SteeringLimits.STEERING_FRONT_RIGHT

	if motor_sel == MotorSelect.STEERING_BACK_LEFT:
		return SteeringLimits.STEERING_BACK_LEFT
	if motor_sel == MotorSelect.STEERING_BACK_RIGHT:
		return SteeringLimits.STEERING_BACK_RIGHT

def SetWheelAngleDirectly(motor_sel, angle, speed): # assumes we get the angle in radians
	limits = MapMotorSelToLimits(motor_sel)
	offset = (limits[1] + limits[0]) / 2 # our middle position should equal 0 degrees
	target_pos = (limits[1] - limits[0]) / steering_angle_range * angle + offset
	# SetPosBlocking(motor_sel, speed, target_pos)
	target_pos = min(limits[1] - safety_constant, target_pos)
	target_pos = max(limits[0] + safety_constant, target_pos)
	SetPosPID(motor_sel, int(target_pos))

def SetSteeringVelocity(motor_sel, ang_vel): # alternative to the function above. not sure what the kinematic model returns. it assumes we get angular velocity directly as [radians / s]
	limits = MapMotorSelToLimits(motor_sel)
	tick_vel = ang_vel / steering_angle_range * (limits[1] - limits[0])
	SetSpeedRover(motor_sel, tick_vel)

def ConvertMeterToTicks(meter):
	return meter / (wheel_diameter * 2 * 3.1415926535897 / ticks_per_rotation)

def ConvertMperSToEncSpeed(mpers):
	return ConvertMeterToTicks(mpers)

def ConvertTicksToMeters(ticks):
	return ticks / ticks_per_rotation * (wheel_diameter * 2 * 3.1415926535897)

def ConvertEncSpeedToMperS(enc_speed):
	return ConvertTicksToMeters(enc_speed)

def ConvertAbsEncToRad(enc_val, limits):
	offset = (limits[1] + limits[0]) / 2
	enc_range = limits[1] - limits[0]
	return (enc_val - offset) / enc_range * steering_angle_range

#maximum velocity per wheel is ~0.145
def SetWheelVelocity(motor_sel, vel):
	SetSpeedRover(motor_sel, ConvertMperSToEncSpeed(vel))
	#print ConvertMperSToEncSpeed(vel)

def SetSteeringPositionsThreaded(w1, w4, w3, w6):
	turning_speed = 300
	t1 = threading.Thread(target=SetWheelAngleDirectly, args=(MotorSelect.STEERING_FRONT_LEFT, w1, turning_speed))
	t2 = threading.Thread(target=SetWheelAngleDirectly, args=(MotorSelect.STEERING_FRONT_RIGHT, w4, turning_speed))
	t3 = threading.Thread(target=SetWheelAngleDirectly, args=(MotorSelect.STEERING_BACK_LEFT, w3, turning_speed))
	t4 = threading.Thread(target=SetWheelAngleDirectly, args=(MotorSelect.STEERING_BACK_RIGHT, w6, turning_speed))

	t1.start()
	t2.start()
	t3.start()
	t4.start()
	
	
	t1.join()
	t2.join()
	t3.join()
	t4.join()

def SetSteeringPositions(w1, w4, w3, w6):
	SetWheelAngleDirectly(MotorSelect.STEERING_FRONT_LEFT, w1, 100)
	SetWheelAngleDirectly(MotorSelect.STEERING_FRONT_RIGHT, w4, 100)
	#SetWheelAngleDirectly(MotorSelect.STEERING_BACK_LEFT, w3, 100)
	#SetWheelAngleDirectly(MotorSelect.STEERING_BACK_RIGHT, w6, 100)

def MoveRover(w1, w2, w3, w4, w5, w6):
	#AccessRoboclaw()
	#address = 128
	# roboclaw.SpeedM1M2(address, int(w3.linear.x), int(w2.linear.x))
	###########################################################	
	#SetWheelVelocity(MotorSelect.WHEEL_FRONT_LEFT, w1.linear.x)
	#SetWheelVelocity(MotorSelect.WHEEL_CENTER_LEFT, w2.linear.x)
	#SetWheelVelocity(MotorSelect.WHEEL_BACK_LEFT, w3.linear.x)
	#SetWheelVelocity(MotorSelect.WHEEL_FRONT_RIGHT, w4.linear.x)
	#SetWheelVelocity(MotorSelect.WHEEL_CENTER_RIGHT, w5.linear.x)
	#SetWheelVelocity(MotorSelect.WHEEL_BACK_RIGHT, w6.linear.x)
############################################################
	testspeed = 0.02
	SetWheelVelocity(MotorSelect.WHEEL_FRONT_LEFT, testspeed)
	SetWheelVelocity(MotorSelect.WHEEL_CENTER_LEFT, testspeed)
	SetWheelVelocity(MotorSelect.WHEEL_BACK_LEFT, testspeed)
	SetWheelVelocity(MotorSelect.WHEEL_FRONT_RIGHT, testspeed)
	SetWheelVelocity(MotorSelect.WHEEL_CENTER_RIGHT, testspeed)
	SetWheelVelocity(MotorSelect.WHEEL_BACK_RIGHT, testspeed)
	# SetSteeringPositions(-15.0 / 180.0 * 3.141, 0, 0, 0) # just for testing
#################################################
	#SetSteeringPositions(w1.angular.z, w4.angular.z, w3.angular.z, w6.angular.z)
###########################################
	#SetSteeringPositionsThreaded(w1.angular.z, w4.angular.z, w3.angular.z, w6.angular.z)
	#SetSteeringVelocity(MotorSelect.STEERING_FRONT_LEFT, w1.angular.z)
	#SetSteeringVelocity(MotorSelect.STEERING_FRONT_RIGHT, w4.angular.z)
	#SetSteeringVelocity(MotorSelect.STEERING_BACK_LEFT, w3.angular.z)
	#SetSteeringVelocity(MotorSelect.STEERING_BACK_RIGHT, w6.angular.z)

	
	#print "wheel 3 and 2"+ " " +  str(int(w3.linear.x)) + " " + str(int(w2.linear.x))
	#roboclaw.SpeedM1M2(address, 0, 0)

	#address = 129
#	roboclaw.SpeedM1M2(address, int(w1.linear.x), int(w4.linear.x))
	#print "wheel 1 and 4"+ " " +  str(int(w1.linear.x)) + " " + str(int(w4.linear.x))
	#roboclaw.SpeedM1M2(address, 0, 0)


	#address = 130
#	roboclaw.SpeedM1M2(address, int(w5.linear.x), int(w6.linear.x))
	#print "wheel 5 and 6"+ " " +  str(int(w5.linear.x)) + " " + str(int(w6.linear.x))
	#roboclaw.SpeedM1M2(address, 0, 0)
	
	#address = 131
	#print "wheel 3 and 1"+ " " +  str(int(w1.angular.z)) + " " + str(int(w3.angular.z))
#	roboclaw.SpeedM1M2(address, int(w1.angular.z), int(w3.angular.z))	
	#w6 -> 131 : 1
	#roboclaw.SpeedM1M2(address, 0, 0)	

	#address = 132
#	roboclaw.SpeedM1M2(address, int(w4.angular.z), int(w6.angular.z))
	#w1 -> 132 : 1
	#w3 -> 132 : 2
	#roboclaw.SpeedM1M2(address, 0, 0)
	#print "wheel 4 and 6"+ " " +  str(int(w4.angular.z))+ " " +str(int(w6.angular.z))
	#LeaveRoboclaw();

def StopRover():
	AccessRoboclaw()
	addresses = [128, 129, 130]
	for a in addresses:
		roboclaw.SpeedM1M2(a, 0, 0)
		roboclaw.ResetEncoders(a)
		
	addresses = [131, 132, 133]
	for a in addresses:
		roboclaw.SpeedM1M2(a, 0, 0)
	#LeaveRoboclaw()

#Steering
def CalibrateAbsoluteEncoders(): #better use the roboclaw software for that
	AccessRoboclaw()
	roboclaw.ResetEncoders(131)
	cal_speed = 100
	#drive to positive limit pos
	roboclaw.SpeedM1M2(131, cal_speed, cal_speed)
	time.sleep(2)
	roboclaw.SpeedM1M2(132, cal_speed, cal_speed)
	time.sleep(2) # wait till position is reached
	roboclaw.SpeedM1M2(131, 0, 0)
	time.sleep(2)
	roboclaw.SpeedM1M2(132, 0, 0)
	time.sleep(2)

	#get limit positions
	while True:
		enc = GetEncRover(MotorSelect.STEERING_FRONT_LEFT)
		if enc[1] != 0:
			break
	print "upper limit w1: " + str(enc);

	while True:
		enc = GetEncRover(MotorSelect.STEERING_FRONT_RIGHT)
		if enc[1] != 0:
			break
	print "upper limit w4: " + str(enc);

	while True:
		enc = GetEncRover(MotorSelect.STEERING_BACK_LEFT)
		if enc[1] != 0:
			break
	print "upper limit w3: " + str(enc);

	while True:
		enc = GetEncRover(MotorSelect.STEERING_BACK_RIGHT)
		if enc[1] != 0:
			break
	print "upper limit w6: " + str(enc);


	#drive to positive limit pos
	roboclaw.SpeedM1M2(131, -cal_speed, -cal_speed)
	time.sleep(2)
	roboclaw.SpeedM1M2(132, -cal_speed, -cal_speed)
	time.sleep(2) # wait till position is reached
	roboclaw.SpeedM1M2(131, 0, 0)
	time.sleep(2)
	roboclaw.SpeedM1M2(132, 0, 0)
	time.sleep(2)

	#get limit positions
	while True:
		enc = GetEncRover(MotorSelect.STEERING_FRONT_LEFT)
		if enc[1] != 0:
			break
	print "lower limit w1: " + str(enc);

	while True:
		enc = GetEncRover(MotorSelect.STEERING_FRONT_RIGHT)
		if enc[1] != 0:
			break
	print "lower limit w4: " + str(enc);

	while True:
		enc = GetEncRover(MotorSelect.STEERING_BACK_LEFT)
		if enc[1] != 0:
			break
	print "lower limit w3: " + str(enc);

	while True:
		enc = GetEncRover(MotorSelect.STEERING_BACK_RIGHT)
		if enc[1] != 0:
			break
	print "lower limit w6: " + str(enc);




	#LeaveRoboclaw()

#wheel rotation
encWheelList = []
def ReadIncrementalEncoders():

	AccessRoboclaw()
	addresses = [128, 129, 130, 131, 132]
	encWheelList = [];

	for address in addresses:
		#print str(roboclaw.ReadError(address))
		while True:
			# enc_1 = roboclaw.ReadSpeedM1(address)
			enc_1 = roboclaw.ReadEncM1(address)
			break
			if enc_1[0] == 1 and enc_1[2] == 130:
				#print str(address) + "M1:" + str(enc_1)
				break
			break
		
		while True:
			#enc_2 = roboclaw.ReadSpeedM2(address)
			enc_2 = roboclaw.ReadEncM2(address)
			break
			if len(enc_2) > 2:
				#print enc_1
				break
		#if len(enc_1) == 3:
		#	print enc_1
		#if len(enc_2) == 3:
		#	print enc_2

		encWheelList.append(enc_1)
		encWheelList.append(enc_2)
		#print "enc2" + " " + str(enc2)
		#print str(address) + ": " + str(enc_1) + " " + str(enc_2)
	print encWheelList
	# print "----------end"
	#LeaveRoboclaw()

def publishValidLinVel(pub, motor_sel):
	enc_val = GetEncSpeed(motor_sel)
	if enc_val[0] == 1:
		w = Twist()
		w.linear.x = ConvertTicksToMeters(enc_val[1])
		w.angular.z = float('NaN')
		pub.publish(w)

def publishValidAngPos(pub, motor_sel):
	enc_val = GetEncRover(motor_sel)
	if enc_val[0] == 1:
		w = Twist()
		w.angular.z = ConvertAbsEncToRad(enc_val[1], MapMotorSelToLimits(motor_sel))
		w.linear.x = float('NaN')
		pub.publish(w)

keep_publishing = True
def PublishEncValues(pub1, pub2, pub3, pub4, pub5, pub6):
	while keep_publishing:
		publishValidLinVel(pub1, MotorSelect.WHEEL_FRONT_LEFT)
		publishValidAngPos(pub1, MotorSelect.STEERING_FRONT_LEFT)

		publishValidLinVel(pub2, MotorSelect.WHEEL_FRONT_RIGHT)
		publishValidAngPos(pub2, MotorSelect.STEERING_FRONT_RIGHT)

		publishValidLinVel(pub3, MotorSelect.WHEEL_CENTER_LEFT)

		publishValidLinVel(pub4, MotorSelect.WHEEL_CENTER_RIGHT)

		publishValidLinVel(pub5, MotorSelect.WHEEL_BACK_LEFT)
		publishValidAngPos(pub5, MotorSelect.STEERING_BACK_LEFT)

		publishValidLinVel(pub6, MotorSelect.WHEEL_BACK_RIGHT)
		publishValidAngPos(pub6, MotorSelect.STEERING_BACK_RIGHT)


successfully_read = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
def ReadEncodersTest():
	#while True:
		#time.sleep(1)
		#print roboclaw.ReadMainBatteryVoltage(128)
	encoder_list = []
	time.sleep(0.01)
	encoder_list.append(GetEncSpeed(MotorSelect.WHEEL_FRONT_LEFT))
	if encoder_list[-1][0] == 1:
		successfully_read[0] = successfully_read[0] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncSpeed(MotorSelect.WHEEL_FRONT_RIGHT))
	if encoder_list[-1][0] == 1:
		successfully_read[1] = successfully_read[1] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncSpeed(MotorSelect.WHEEL_CENTER_LEFT))
	if encoder_list[-1][0] == 1:
		successfully_read[2] = successfully_read[2] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncSpeed(MotorSelect.WHEEL_CENTER_RIGHT))
	if encoder_list[-1][0] == 1:
		successfully_read[3] = successfully_read[3] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncSpeed(MotorSelect.WHEEL_BACK_LEFT))
	if encoder_list[-1][0] == 1:
		successfully_read[4] = successfully_read[4] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncSpeed(MotorSelect.WHEEL_BACK_RIGHT))
	if encoder_list[-1][0] == 1:
		successfully_read[5] = successfully_read[5] + 1

	time.sleep(0.01)
	# steering
	encoder_list.append(GetEncRover(MotorSelect.STEERING_FRONT_LEFT))
	if encoder_list[-1][0] == 1:
		successfully_read[6] = successfully_read[6] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncRover(MotorSelect.STEERING_FRONT_RIGHT))
	if encoder_list[-1][0] == 1:
		successfully_read[7] = successfully_read[7] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncRover(MotorSelect.STEERING_BACK_LEFT))
	if encoder_list[-1][0] == 1:
		successfully_read[8] = successfully_read[8] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncRover(MotorSelect.STEERING_BACK_RIGHT))
	if encoder_list[-1][0] == 1:
		successfully_read[9] = successfully_read[9] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncRover(MotorSelect.CAM_PAN))
	if encoder_list[-1][0] == 1:
		successfully_read[10] = successfully_read[10] + 1

	time.sleep(0.01)
	encoder_list.append(GetEncRover(MotorSelect.CAM_TILT))
	if encoder_list[-1][0] == 1:
		successfully_read[11] = successfully_read[11] + 1

	print encoder_list
	print "read successful: " + str(successfully_read)

def keyboard_control(mot_sel, speed):
	
	
	key = input("direction?")
	if key == 1:
		SetSpeedRover(mot_sel, speed)
	elif key == 2:
		SetSpeedRover(mot_sel, -speed)
	else:
		SetSpeedRover(mot_sel, 0)
		while True:
			speed = GetEncRover(MotorSelect.CAM_PAN)
			if speed[0] == 1:
				print speed
				break
def tiltCam(radians):
	SetWheelAngleDirectly(MotorSelect.CAM_TILT, radians, 42)


def turnCam(dir):
	if dir > 0:
		SetSpeedRover(MotorSelect.CAM_PAN, 50)
		time.sleep(0.2)
		SetPosPID(MotorSelect.CAM_PAN, 500)
	else:
		SetSpeedRover(MotorSelect.CAM_PAN, 50)
		time.sleep(0.2)
		SetPosPID(MotorSelect.CAM_PAN, 882)
		

if __name__ == '__main__':
	
	sub1 = rospy.Subscriber('beetle_left_wheel_speed', Twist, leftwheelCallback)
	sub2 = rospy.Subscriber('beetle_center_left_wheel_speed', Twist, centerleftwheelCallback)
	sub3 = rospy.Subscriber('beetle_rear_left_wheel_speed', Twist, rearleftwheelCallback)
	sub4 = rospy.Subscriber('beetle_right_wheel_speed', Twist, rightwheelCallback)
	sub5 = rospy.Subscriber('beetle_center_right_wheel_speed', Twist, centerrightwheelCallback)
	sub6 = rospy.Subscriber('beetle_rear_right_wheel_speed', Twist, rearrightwheelCallback)

	pub1 = rospy.Publisher('beetle_front_left_wheel_converted', Twist, queue_size=100) # wheel velocity in m/s
	pub2 = rospy.Publisher('beetle_front_right_wheel_converted', Twist, queue_size=100) # wheel velocity in m/s
	pub3 = rospy.Publisher('beetle_center_left_wheel_converted', Twist, queue_size=100) # wheel velocity in m/s
	pub4 = rospy.Publisher('beetle_center_right_wheel_converted', Twist, queue_size=100) # wheel velocity in m/s
	pub5 = rospy.Publisher('beetle_rear_left_wheel_converted', Twist, queue_size=100) # wheel velocity in m/s
	pub6 = rospy.Publisher('beetle_rear_right_wheel_converted', Twist, queue_size=100) # wheel velocity in m/s
	
	rospy.init_node('beetle_control', anonymous=True)
	#rate = rospy.Rate(10)
	StopRover()



