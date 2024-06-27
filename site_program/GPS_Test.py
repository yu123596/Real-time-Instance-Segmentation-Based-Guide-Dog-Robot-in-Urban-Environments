import numpy as np
import sys
import time
import cv2
import os 
import pyzed.sl as sl
import keyboard
import serial
import math
import googlemaps
from pypolyline.cutil import decode_polyline
import gmplot
from haversine import haversine, Unit
import matplotlib.pyplot as plt
import csv
from random import sample

key="AIzaSyC_kBNkeFbIhz155RVDMvl8PEiH0VObMlc"
gmaps=googlemaps.Client(key)
origin = "24.1796933, 120.6474330"
destination = "24.179035,120.651736" #school back door
mode = "walking"
traffic_model = "optimistic"
routes = gmaps.directions(origin, destination,mode,optimize_waypoints= False)#waypoints,
polyline = (routes[0]["overview_polyline"]["points"]).encode()
path_point0 = decode_polyline(polyline, 5)

gmap = gmplot.GoogleMapPlotter(path_point0[0][1],path_point0[0][0] , 22 , apikey = key)
lat_point = []
lng_point = []
for i in range(len(path_point0)):
    gmap.marker(path_point0[i][1] , path_point0[i][0] , label=str(i) ,color='red')
    lat_point.append(path_point0[i][1])
    lng_point.append(path_point0[i][0])
print("Path point number: {}".format(len(path_point0)))
path_point = list(zip(lat_point,lng_point))
gmap.plot(lat_point , lng_point, 'cornflowerblue', edge_width = 3.0)
gmap.draw('./results/map5.html')