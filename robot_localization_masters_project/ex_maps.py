# -*- coding: utf-8 -*-
"""
Created on Fri Oct  4 00:00:00 2019

@author: datta
"""
from pygmaps import *
import pandas as pd 

ground_truth = pd.read_csv("CSVFiles/ground_truth_maps.csv")
lat_lon = pd.DataFrame(columns=['latitude', 'longitude'])
lat_lon['latitude'] = ground_truth['Latitude']
lat_lon['longitude'] = ground_truth['Longitude']


gps = pd.read_csv("CSVFiles/gps_fix.csv")

gps_lat_lon = pd.DataFrame(columns=['latitude', 'longitude'])
gps_lat_lon['latitude'] = gps['.latitude']
gps_lat_lon['longitude'] = gps['.longitude']

mymap1 = maps(47.059048, 15.458519, 100) 

#print(lat_lon.values.tolist())

latitude_list = lat_lon['latitude']
  
# list of longitudes 
longitude_list = lat_lon['longitude'] 
  
mymap1.addpath(lat_lon.values.tolist(), color = "# 00FF00")
mymap1.addpath(gps_lat_lon.values.tolist(), color = "0000FF")

# for i in range(len(latitude_list)): 
  
#     # add a point into a map 
#     # 1st argument is latitude 
#     # 2nd argument is longitude 
#     # 3rd argument is colour of the point showed in thed map 
#     # using HTML colour code e.g. 
#     # red "# FF0000", Blue "# 0000FF", Green "# 00FF00" 
#     mymap1.addpoint(latitude_list[i], longitude_list[i], "# FF0000") 
      
mymap1.draw('map.html') 
  
