# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 00:54:39 2019

@author: datta
"""
import numpy as np
import pandas as pd
import csv as csv
import matplotlib.pyplot as plt
from stats import * 
class Plot:
    def __init__(self, filename, x_axis, y_axis, filename2=None, x_axis2=None, y_axis2=None):
        self.filename = filename
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.filename2 = filename2
        self.x_axis2 = x_axis2
        self.y_axis2 = y_axis2

    def LoadCsvAndPlot(self):
        readFile = pd.read_csv(self.filename)
        plt.plot(readFile[self.x_axis], readFile[self.y_axis])
    
    def LoadTwoCsvAndPlot(self):
        readfile1 = pd.read_csv(self.filename)
        readfile2 = pd.read_csv(self.filename2)
        plt.plot(readfile1[self.x_axis], readfile1[self.y_axis])
        plt.plot(readfile2[self.x_axis2], readfile2[self.y_axis2])

    def ShowPlot(self):
        plt.show()


"""
Test and Debug:

#Create Ground truth file object and plot
plot5 = Plot("CSVFiles/ground_truth.csv", "Easting", "Northing")
plot5.LoadCsvAndPlot()
plot5.ShowPlot()


#Husky odom

plot4 = Plot("CSVFiles/husky_odom.csv", ".pose.pose.position.x", ".pose.pose.position.y")
plot4.LoadCsvAndPlot()
plot4.ShowPlot()




plot4 = Plot("CSVFiles/zed_odom.csv", ".pose.pose.position.x", ".pose.pose.position.y")
plot4.LoadCsvAndPlot()
plot4.ShowPlot()

"""

"""
#Create Gps file object and plot
plot1 = Plot("CSVFiles/gps_fix.csv", ".latitude", ".longitude", filename2="CSVFiles/ground_truth.csv", x_axis2="Latitude", y_axis2="Longitude")
plot1.LoadTwoCsvAndPlot()
plot1.ShowPlot()
"""

"""
#Create Gps file object and plot
plot1 = Plot("CSVFiles/ground_truth.csv", "Easting", "Northing", filename2="CSVFiles/bag/odometry-filtered.csv", x_axis2=".pose.pose.position.x", y_axis2=".pose.pose.position.y")
plot1.LoadTwoCsvAndPlot()
plot1.ShowPlot()
"""

