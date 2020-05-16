# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 00:54:39 2019

@author: datta
"""
###########################################################################
# Error State Extended Kalman Filter for mobile robot Localization
#
# Fusion of IMU and GPS
#
# Ground truth file:
# IMU file:
# GPS measurement file:
#
###########################################################################

#Import external libraries
import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd
from numpy.linalg import inv 
import utm
from stats import *
#Import custom libraries
from DataConvert import *
from Plot import * 
from rotations import *

############################################################################
#------------------------Load Data-----------------------------------------#
ground_truth = pd.read_csv("CSVFiles/ground_truth.csv")

imu = pd.read_csv("CSVFiles/imu_data.csv")
gps = pd.read_csv("CSVFiles/gps_fix.csv")

R = rpy_to_mat(0.0, -1.5708, 3.1416 )
T = np.array([[0.19, 0.0, 0.149]])

TF = np.concatenate((R, T.T), axis=1)
Homogeneous_TF = np.concatenate((TF, np.array([[0, 0, 0, 1]])), axis=0)

#Acceleration components loaded to dataframe imu_f i.e imu_forces from imu topic imu_data.csv file.
imu_f = pd.DataFrame(columns=['acc_x', 'acc_y', 'acc_z'])
imu_f['acc_y'] = imu['.linear_acceleration.x']
imu_f['acc_x'] = imu['.linear_acceleration.y']
imu_f['acc_z'] = -imu['.linear_acceleration.z']
#imu_f['w'] = np.ones(np.shape(imu_f.shape[0]))

print(imu_f.iloc[0])

"""
a = np.array([1])
for k in range(0, imu_f.shape[0]):
    x = np.concatenate((imu_f.iloc[k], a), axis=0)
    y = Homogeneous_TF @ x
    imu_f.iloc[k] = y[0:3]

print(imu_f.iloc[0])
"""

#Angular velocity components loaded to dataframe imu_w ie imu_angular from imu ros topic imu_data.csv file
imu_w = pd.DataFrame(columns=['ang_vel_x', 'ang_vel_y', 'ang_vel_z'])
imu_w['ang_vel_y'] = imu['.angular_velocity.x']
imu_w['ang_vel_x'] = imu['.angular_velocity.y']
imu_w['ang_vel_z'] = -imu['.angular_velocity.z']

#imu_w = Homogeneous_TF @ imu_w 

#save time of Imu data
imu_time = imu['time']
imu_time_utm = imu['.header.stamp.secs'] 
time_utc_gps = list(gps['.header.stamp.secs'])


#Load GPS data
gps_lat = list(gps['.latitude'])
gps_lon = list(gps['.longitude'])
gps_height = list(gps['.altitude'])

#### 2. Constants ##############################################################################
#Set covariance

"""
var_imu_f = 10
var_imu_w = 10
var_gnss  = 2
"""

var_imu_f = 0.01
var_imu_w = 0.01
var_gnss  = 2


#Constants

g = np.array([0, 0, -9.81])  # gravity
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)  # motion model noise jacobian

#print('l_jac',l_jac)

h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)  # measurement model jacobian

#print('h_jac', h_jac)
####  Initial Values #########################################################################

p_est = np.zeros([imu_f.shape[0], 3])  # position estimates
v_est = np.zeros([imu_f.shape[0], 3])  # velocity estimates
q_est = np.zeros([imu_f.shape[0], 4])  # orientation estimates as quaternions
p_cov = np.zeros([imu_f.shape[0], 9, 9])  # covariance matrices at each timestep

# Set initial values.
p_est[0] = np.array([ground_truth['Easting'].iloc[0], ground_truth['Northing'].iloc[0], 0.0])
print('p_est_init', p_est[0])


v_est[0] = np.array([0, 0, 0])
print('v_est_init', v_est[0])

euler_angles = np.array([ground_truth['Roll'].iloc[0], ground_truth['Pitch'].iloc[0], wraptopi(ground_truth['Heading'].iloc[0])])
q_est[0] = Quaternion(euler=euler_angles).to_numpy()
print('q_est_init', q_est[0])

p_cov[0] = np.zeros(9)  # covariance of estimate - Initial estimates covariance is 0 because we are absolutely sure about the initial estimated postion (i.e from Ground truth) 
prev_timestamp = 0
X_gps = []
Y_gps = []

x_hat_meas = []
p_hat_meas = []
counter =0

error_in_estimation= []
error_in_GPS = []
#gps_noise_cov = np.array
"""
gps_noise_cov = np.zeros([3, 9])
gps_noise_cov[:, :3] = np.eye(3) * var_gnss
gps_noise_cov[:, 3:6] = np.eye(3) * 999999
gps_noise_cov[:, 6:9] = np.eye(3) * 999999
"""


########## Measurement Update ############################################################
def measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check):
    X_gps.append(y_k[0])
    Y_gps.append(y_k[1])
    Measurement = h_jac @ np.array([y_k[0], y_k[1], y_k[2], 0., 0., 0., 0., 0., 0.]).T #+ sensor_var
    #print('Measurement: ', Measurement, 'Measurement shape', np.shape(Measurement))
    
    # 3.1 Compute Kalman Gain
    K = p_cov_check @ h_jac.T @ inv(h_jac @ p_cov_check @ h_jac.T + np.eye(3)*sensor_var)
    #print('Kalman Gain:', K, 'Kalman Gain shape:', np.shape(K))

    # 3.2 Compute error state
    Error_state = K @ (Measurement - p_check)
    #print('Error State:', Error_state, 'Error state shape:', np.shape(Error_state))
    # 3.3 Correct predicted state
    
    p_hat = p_check + Error_state[0:3]
    #print('Position estimates: ', p_hat)

    #a = np.array([1])

    #p_hat = Homogeneous_TF @ np.concatenate((p_hat, a), axis=0)

    #p_hat = p_hat[0:3]
    
    v_hat = v_check + Error_state[3:6]
    #print('Velocity estimates: ', v_hat)

    q_hat = Quaternion(axis_angle=Error_state[6:9]).quat_mult_right(q_check)
    #print('Orientation estimates: ', q_hat)

    # 3.4 Compute corrected covariance
    p_cov_hat = (np.eye(9) - K @ h_jac) @ p_cov_check
    #print("Corrected Covariance: ", p_cov_hat, "shape: ", np.shape(p_cov_hat))

    return p_hat, v_hat, q_hat, p_cov_hat

############ Main Filter Loop #####################################################################

for k in range(1, imu_f.shape[0]):  # start at 1 b/c we have initial prediction from gt
    delta_t = ros_time_diff(imu_time[k], imu_time[k - 1])

    # 1. Update state with IMU inputs
    """
    a = np.array([1])
    x = np.concatenate((imu_f.iloc[k-1], a), axis=0)
    y = Homogeneous_TF @ x
    imu_f.iloc[k-1] = y[0:3]

    x = np.concatenate((imu_w.iloc[k-1], a), axis=0)
    y = Homogeneous_TF @ x
    imu_w.iloc[k-1] = y[0:3]
    """
    p = p_est[k-1]
    v = v_est[k-1]
    q = q_est[k-1]

    #print('p_est::', p, 'p_est shape:', np.shape(p))
    #print('v_est:', v, 'v_est shape:', np.shape(v))

    C_ns = Quaternion(*q).to_mat()
    #print('C_ns:', C_ns, 'C_ns shape:', np.shape(C_ns))

    p_check = p + delta_t * v + 0.5 * delta_t**2 * (C_ns @ imu_f.iloc[k-1] + g)
    #print('p_check:', p_check, 'p_check shape:', np.shape(p_check))

    v_check = v + delta_t * (C_ns @ imu_f.iloc[k-1] + g)
    #print('v_check:', v_check, 'v_check shape:', np.shape(v_check))

    ang_vel = np.array([imu_w['ang_vel_x'].iloc[k-1], imu_w['ang_vel_y'].iloc[k-1], imu_w['ang_vel_z'].iloc[k-1]])

    q_check = Quaternion(axis_angle=ang_vel * delta_t).quat_mult_left(q)
    #print('q_check:', q_check, 'q_check shape:', np.shape(q_check))

    # 1.1 Linearize the motion model and compute Jacobians
    F = np.eye(9)
    F[0:3,3:6] = np.eye(3) * delta_t
    F[3:6,6:9] = -C_ns @ skew_symmetric(imu_f.iloc[k-1]) * delta_t
    #print("F matrix:", F, "shape:", np.shape(F))

    # 2. Propagate uncertainty
    L = l_jac
    Q = delta_t**2 * np.diag([var_imu_f, var_imu_f, var_imu_f, var_imu_w, var_imu_w, var_imu_w])
    P_cov_check = F @ p_cov[k-1] @ F.T + L @ Q @ L.T
    #print("P_cov_check:", P_cov_check, "P_cov_check shape:", np.shape(P_cov_check))

    utc_current_timestamp = imu_time_utm[k]

    """
    if prev_timestamp!= utc_current_timestamp and utc_current_timestamp in time_utc_gps:
    #if utc_current_timestamp in time_utc_gps:
        prev_timestamp = utc_current_timestamp
        gps_index = time_utc_gps.index(utc_current_timestamp)
        utm_co = (utm.from_latlon(gps_lat[gps_index], gps_lon[gps_index]))
        gnss_data = np.array([utm_co[0], utm_co[1], 0])
        p_check, v_check, q_check, P_cov_check = measurement_update(var_gnss, P_cov_check, gnss_data, p_check, v_check, q_check)
        counter = counter + 1
        if counter == 10:
            x_hat_meas.append(p_check)
            p_hat_meas.append(P_cov_check)
            counter = 0
        #error_in_estimation.append(np.sqrt((ground_truth['Easting'][groundtruth_index]-X_check[0])**2+(ground_truth['Northing'][groundtruth_index]-X_check[1])**2))
        #error_in_GPS.append(np.sqrt((ground_truth['Easting'][groundtruth_index]-utm_co[0])**2+(ground_truth['Northing'][groundtruth_index]-utm_co[1])**2))
    """
    
    # Update states (save)
    p_est[k] = p_check
    v_est[k] = v_check
    q_est[k] = q_check
    p_cov[k] = P_cov_check



e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(p_est[:,0], p_est[:,1], label='Estimated')
ax.plot(X_gps, Y_gps ,label='GPS')
ax.plot(ground_truth['Easting'], ground_truth['Northing'], label='Ground Truth')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Vehicle trajectory (UTM)')
plt.legend()
plt.show()


"""
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
size_meas = np.shape(x_hat_meas)
"""
"""
for k in range(0, size_meas[0]):
    plot_covariance_ellipse((x_hat_meas[k][0], x_hat_meas[k][1]), p_hat_meas[k][0:2, 0:2])
plt.show()
"""
