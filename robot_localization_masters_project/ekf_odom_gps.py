# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 00:54:39 2019

@author: datta
"""
###########################################################################
# Extended Kalman Filter for mobile robot Localization
#
# Fusion of wheel odometry or Visual Odometry and GPS
#
# Ground truth file:
# Wheel odometry file:
# Visual odometry file:
# GPS measurement file:
#
###########################################################################

#Import external libraries
import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd
from numpy.linalg import inv 
import utm
#Import custom libraries
from stats import * 
from DataConvert import *
from Plot import * 
from rotations import *

#load Ground truth data
ground_truth = pd.read_csv("CSVFiles/ground_truth.csv")
#print("ground truth data:", ground_truth)

wheel_odom = pd.read_csv("CSVFiles/husky_odom.csv")
#wheel_odom = pd.read_csv("CSVFiles/zed_odom.csv")#TODO:Configure visual odometry path for csv file
#print("Wheel odometry measurements:", wheel_odom)


gps_meas = pd.read_csv("CSVFiles/gps_fix.csv")
#print("Gps measuremnets:", gps_meas)

######################################################################################
#load meaurement data from dataframe to column list----------------------------------#
######################################################################################
time_utc_odom = wheel_odom['.header.stamp.secs']
time_local_odom = wheel_odom['time']
time_utc_gps = list(gps_meas['.header.stamp.secs'])
time_utc_groundtruth = list(ground_truth['UTC'])

x_meas = wheel_odom['.pose.pose.position.x']
y_meas = wheel_odom['.pose.pose.position.y']
gps_meas_lat = list(gps_meas['.latitude'])
gps_meas_lon = list(gps_meas['.longitude'])
gps_meas_height = list(gps_meas['.altitude'])
#t = ros_time_df_To_Seconds(wheel_odom['.header.stamp.secs'])

#Use X-ECEF, Y-ECEF of ground truth
x_init = ground_truth['Easting'].iloc[0]
y_init = ground_truth['Northing'].iloc[0]
#theta_init = wraptopi(np.arctan2(x_init, y_init))
#theta_init = wraptopi(ground_truth['Heading'].iloc[0] * np.pi/180) #TODO:Check if this is O
theta_init = wraptopi(ground_truth['Heading'].iloc[0]) #TODO:Check if this is O


print("x_init:", x_init, "y_init:", y_init)

#print("len:", len(wheel_odom), "len gps:", len(gps_meas))

#Initialize estimation matrix with zeros 
X_est = np.zeros([len(wheel_odom), 3])
P_est = np.zeros([len(wheel_odom), 3, 3])

X_est[0] = np.array([x_init, y_init, theta_init])
P_est[0] = np.diag([100.0, 100.0, 10.0])

#print("X_est:", X_est, "\n", "P_est:", P_est)

#trans_var = 0.001  # translation velocity variance  
#rot_var = 0.0001  # rotational velocity variance 
#gps_var = 5 #GPS measurement variance

trans_var = 0.001  # translation velocity variance 
rot_var = 0.0001  # rotational velocity variance 
gps_var = 5 #GPS measurement variance

Q = np.diag([trans_var, rot_var])

#gps time stamp previous
prev_timestamp = 0

H = np.zeros([3,3])

#Only X and Y of state vector are updated 
H[0:2, 0:2] = np.eye(2) 

#Error state extended kalman filter
ES_EKF = 1
X_gps = []
Y_gps = []

X_odom = []
Y_odom = []

"""
gnss_var = np.zeros([3, 3])
gnss_var[0:2,0:2] = np.eye(2) * gps_var
gnss_var[2, 2] = 999999
"""
error_in_estimation = []
error_in_GPS = []
error_in_odometry = []
counter =0 
x_hat_meas = []
p_hat_meas = []

#-----------------------------------------Measurement update----------------------------------------------#

#GPS measurement update
def measurement_update_GPS(X_check, p_check, x, y):
    X_gps.append(x)
    Y_gps.append(y)

    measurement = H @ np.array([x, y, 0]).T #+ gps_var

    #print('measurement', measurement)

    #1. Compute Kalman Gain
    K = p_check @ H.T @ inv(H @ p_check @ H.T + np.eye(3) * gps_var)

    #print('K',K, 'K shape:', np.shape(K)) 
    expected = X_check
    if not ES_EKF:
        expected[2] = 0
    #2.Compute errror state
    Error_state = K @ (measurement - expected) 
    X_hat = X_check + Error_state
        
    #
    #3.Compute P_hat

    p_hat = (np.eye(3) - K @ H) @ p_check 

    #print('x_hat', X_hat)
    #print('p_hat', p_hat)


    return X_hat, p_hat


#-----------------------------------------Main Filter loop----------------------------------------------#

for k in range(1, len(time_utc_odom)):

    #Take time difference from local time
    delta_t = ros_time_diff(time_local_odom[k], time_local_odom[k-1])

    #1.Motion model updates from odometry readings (Normalize angles) 
    heading_from = wraptopi(odometry_msg_to_yaw(wheel_odom.iloc[k-1]))
    heading_to = wraptopi(odometry_msg_to_yaw(wheel_odom.iloc[k]))

    trans_vel = np.sqrt((x_meas[k] - x_meas[k-1])**2 + (y_meas[k] - y_meas[k-1])**2) 
    rot_vel_1 = wraptopi(wraptopi(wraptopi(np.arctan2((y_meas[k] - y_meas[k-1]), (x_meas[k] - x_meas[k-1]))) - heading_from))
    rot_vel_2 = wraptopi(heading_to - heading_from - rot_vel_1)

    x_check = X_est[k-1, 0] + delta_t * trans_vel * wraptopi(np.cos(wraptopi(X_est[k-1,2] + rot_vel_1)))
    y_check = X_est[k-1, 1] + delta_t * trans_vel * wraptopi(np.sin(wraptopi(X_est[k-1,2] + rot_vel_1)))
    yaw_check = wraptopi(X_est[k-1, 2] + rot_vel_1 + rot_vel_2)


    #2.Motion model jacobian with respect to last state
    F = np.array(
    [
        [1, 0, -delta_t * trans_vel * wraptopi(np.sin(wraptopi(X_est[k-1,2] + rot_vel_1)))],
        [0, 1, delta_t * trans_vel * wraptopi(np.cos(wraptopi(X_est[k-1,2] + rot_vel_1)))],
        [0, 0, 1]
    ])

    #print("F:", F, np.shape(F))


    #3.Motion model jacobian with respect to noise

    L = np.array(
    [
        [wraptopi(np.cos(wraptopi(X_est[k-1,2]))), 0],
        [wraptopi(np.sin(wraptopi(X_est[k-1,2]))), 0],
        [0, 1]
    ])
    
    X_odom.append(x_check)
    Y_odom.append(y_check)

    #4.Propagate uncertainty
    p_check = F @ P_est[k-1] @ F.T + L @ Q @ L.T
    X_check = np.array([x_check, y_check, wraptopi(yaw_check)])

    #5.Check for GPS measurements
    utc_current_timestamp = time_utc_odom[k]

    groundtruth_index = time_utc_groundtruth.index(utc_current_timestamp)

    
    #if prev_timestamp!= utc_current_timestamp and utc_current_timestamp in time_utc_gps:
    #prev_timestamp = utc_current_timestamp
    gps_index = time_utc_gps.index(utc_current_timestamp)
    #x, y, z = Geodetic_to_ECEF(gps_meas_lat[gps_index], gps_meas_lon[gps_index], gps_meas_height[gps_index])
    utm_co = (utm.from_latlon(gps_meas_lat[gps_index], gps_meas_lon[gps_index]))
    X_check, p_check = measurement_update_GPS(X_check, p_check, utm_co[0], utm_co[1]) 
    counter = counter+1
    if counter == 10:
        x_hat_meas.append(X_check)
        p_hat_meas.append(p_check)
        counter = 0
    
    error_in_estimation.append(np.sqrt((ground_truth['Easting'][groundtruth_index]-X_check[0])**2+(ground_truth['Northing'][groundtruth_index]-X_check[1])**2))
    error_in_GPS.append(np.sqrt((ground_truth['Easting'][groundtruth_index]-utm_co[0])**2+(ground_truth['Northing'][groundtruth_index]-utm_co[1])**2))
    #error_in_odometry.append((X_check[0]-ground_truth['Easting'][groundtruth_index])**2+(X_check[1]-ground_truth['Northing'][groundtruth_index])**2)
    
    #Update state(save it into estimation matrix)
    #print("x_check:", x_check, "y_check:", y_check, "yaw_check:", yaw_check)
    X_est[k] = X_check
    P_est[k] = p_check


#print(X_est)
#print(P_est)

#print("X_odom", X_odom, "Y_odom:", Y_odom)

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(X_est[:, 0], X_est[:, 1], label='Estimated Trajectory')
ax.plot(X_gps, Y_gps ,label='GPS')
ax.plot(ground_truth['Easting'], ground_truth['Northing'], label='Ground Truth')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Vehicle trajectory (UTM)')
plt.legend()
plt.show()

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(X_est[:, 0], X_est[:, 1], label='Estimated Trajectory')
ax.plot(ground_truth['Easting'], ground_truth['Northing'], label='Ground Truth')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Vehicle trajectory (UTM)')
plt.legend()
plt.show()

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(X_est[:, 0], X_est[:, 1], label='Estimated Trajectory')
ax.plot(X_gps, Y_gps ,label='GPS')
#ax.plot(ground_truth['Easting'], ground_truth['Northing'], label='Ground Truth')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Vehicle trajectory (UTM)')
plt.legend()
plt.show()


e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(error_in_estimation, label='error in estimation')
ax.plot(error_in_GPS, label='GPS error')
#ax.plot(error_in_odometry)
ax.set_xlabel('Time [0.1 s]')
ax.set_ylabel('Error [m]')
ax.set_title('Error with respect to Ground truth')
plt.legend()
plt.show()

size = np.shape(X_est)
size_meas = np.shape(x_hat_meas)

for k in range(0, size_meas[0]):
    plot_covariance_ellipse((x_hat_meas[k][0], x_hat_meas[k][1]), p_hat_meas[k][0:2, 0:2])
plt.show()
#for k in range(size[0]-100, size[0]):
    #plot_covariance_ellipse((X_est[k, 0], X_est[k, 1]), P_est[k][0:2, 0:2])

#plt.show()
"""
e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(X_est[:, 2])
ax.set_xlabel('Time [s]')
ax.set_ylabel('theta [rad]')
ax.set_title('Estimated trajectory')
plt.show()
"""
