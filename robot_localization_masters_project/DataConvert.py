# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 00:54:39 2019

@author: datta
"""

from Plot import * 
import pandas as pd 
from rotations import * 
import math
import numpy as np 


# Wraps angle to (-pi,pi] range
def wraptopi(x):
    if x > np.pi:
        x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
    elif x < -np.pi:
        x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
    return x

def ros_time_To_Seconds(ros_time):
    year = ros_time[0:4]
    month = ros_time[5:7]
    day = ros_time[8:10]
    hours = ros_time[11:13]
    minutes = ros_time[14:16]
    seconds = ros_time[17:]

    total_seconds = float(hours) * 3600 + float(minutes) * 60 + float(seconds)
    #print(year, month, day, hours, minutes, seconds, total_seconds)
    return total_seconds

def ros_time_diff(ros_time1, ros_time2):
    time_diff = ros_time_To_Seconds(ros_time1) - ros_time_To_Seconds(ros_time2)
    #print(abs(time_diff))
    return abs(time_diff)

def ros_time_df_To_Seconds(ros_time_df):


    year = ros_time_df[0:4]
    month = ros_time_df[5:7]
    day = ros_time_df[8:10]
    hours = ros_time_df[11:13]
    minutes = ros_time_df[14:16]
    seconds = ros_time_df[17:]

    total_seconds = hours * 3600 + minutes * 60 + seconds
    #print(year, month, day, hours, minutes, seconds, seconds)
    return total_seconds

def odometry_msg_to_euler(odom_msg):
    euler = Quaternion(odom_msg['.pose.pose.orientation.w'],
    odom_msg['.pose.pose.orientation.x'],
    odom_msg['.pose.pose.orientation.y'],
    odom_msg['.pose.pose.orientation.z']).to_euler()
    return euler
    
def odometry_msg_to_yaw(odom_msg):
    yaw = odometry_msg_to_euler(odom_msg)[2]
    return yaw



def Geodetic_to_ECEF(latitude, longitude, height):
    #define a and b
    latitude = math.radians(latitude)
    longitude = math.radians(longitude)
    a = 6378137 #meters
    b = 6356752.31424518 #meters
    N = (a**2)/(np.sqrt(a**2*np.cos(latitude)*np.cos(latitude)+b**2*np.sin(latitude)*np.sin(latitude)))
    x = (N + height) * np.cos(latitude) * np.cos(longitude)
    y = (N + height) * np.cos(latitude) * np.sin(longitude)
    z = ((b**2/a**2)*N + height) * np.sin(latitude)

    return x, y, z

class Converter:
    def __init__(self, column1=None, column2=None):
        self.column1 = column1
        self.column2 = column2
    
    def GPS_to_UTC(self, gps_time):
        #Gps time from Jan 1st 1970 and Unix time is from Jan 5th 1980
        unix_time_diff = 315964782
        utc_time = gps_time + unix_time_diff
        return utc_time

    def UTC_to_GPS(self, unix_time):
        #Gps time from Jan 1st 1970 and Unix time is from Jan 5th 1980
        unix_time_diff = 315964782
        gps_time = unix_time - unix_time_diff
        return gps_time

    def format_to_GPS(self, read_file):
        #print(read_file[self.column1])
        
        ###### Week and GPSTime conversion from GroundTruth data ##############
        #(Week * days_in_week * hours_in_a_day * minutes * seconds + GPS_time)#
        #######################################################################

        gps_time = (read_file[self.column1] * 7 * 24 * 60 * 60 + read_file[self.column2])
        return gps_time

    
"""
### Modified csv file used as Ground truth data

GroundTruth = Converter(column1 = 'Week', column2 = 'GPSTime')
read_file = pd.read_csv("CSVFiles/ground_truth.csv")
gps_time = GroundTruth.format_to_GPS(read_file)
#print(gps_time)
unix_time = GroundTruth.GPS_to_UTC(gps_time)
#print(unix_time)
dataframe = pd.DataFrame(read_file)
dataframe['UTC'] = unix_time
#print(dataframe)
dataframe.to_csv("CSVFiles/ground_truth.csv")

"""


### For debugging date time convestion 
"""

print(ros_time_To_Seconds("2019/07/11/08:54:51.938986"))
print(ros_time_To_Seconds("2019/07/11/08:54:52.938986"))

print(ros_time_diff("2019/07/11/08:54:52.1678494", "2019/07/11/08:54:51.938986"))

"""
"""
### Debug for geo to ecef
x1, y1, z1 = Geodetic_to_ECEF(47.0591225026
, 15.4586022146
, 405.441
)
print(x1, y1, z1)

x2, y2, z2 = Geodetic_to_ECEF(47.0591749850
, 15.4909999
, 405.441
)
print(x2, y2, z2)

print(x1-x2, y1-y2, z1-z2)


#Getting orientation to yaw

read_file = pd.read_csv("CSVFiles/zed_odom.csv")
yaw = odometry_msg_to_yaw(read_file)
dataframe = pd.DataFrame(read_file)
dataframe['yaw'] = yaw
dataframe.to_csv("CSVFiles/zed_odom.csv")
"""

