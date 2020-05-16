#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import serial
import tf

ser = serial.Serial('/dev/beetle_imu', 115200)
number_of_expected_values = 14

def talker():
  sequence = 0
  while not rospy.is_shutdown():
    data = ser.readline()
    data = data.rstrip("\r\n")
    split_data = data.split(', ')

    #rospy.loginfo(data)
    #2555714, -0.01, -0.06, -1.00, 0.61, -0.30, -0.49, -33.01, 46.96, -77.12, -0.0379, 0.9773, -0.2084, -0.0039
    if(len(split_data) == number_of_expected_values):
    	imu = Imu()
    	imu.linear_acceleration.x = float(split_data[1])
    	imu.linear_acceleration.y = float(split_data[2])
    	imu.linear_acceleration.z = float(split_data[3])

    	imu.angular_velocity.x = float(split_data[4])
    	imu.angular_velocity.y = float(split_data[5])
    	imu.angular_velocity.z = float(split_data[6])

    	imu.orientation.w = float(split_data[7])
    	imu.orientation.x = float(split_data[8])
    	imu.orientation.y = float(split_data[9])
    	imu.orientation.z = float(split_data[10])
        
        #imu_broadcaster.sendTransform()
    	imu.header.stamp = rospy.Time.now()
    	imu.header.frame_id = "beetle__imu_link"
    	imu.header.seq = sequence
        #imu.child_frame_id = "beetle__base_link"

    	sequence += 1

    	pub.publish(imu)
    	rospy.sleep(0.001)


if __name__ == '__main__':
  try:
    pub = rospy.Publisher('beetle_imu', Imu, queue_size=100)
    rospy.init_node('beetle_imu_node')
    talker()
  except rospy.ROSInterruptException:
    pass
