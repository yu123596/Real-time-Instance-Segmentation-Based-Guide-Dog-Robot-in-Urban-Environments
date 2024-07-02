# -*- coding: utf-8 -*-
"""
Created on Fri Jun  7 01:18:42 2024

@author: 309-1-RTX3060
"""
import googlemaps
from pypolyline.cutil import decode_polyline
import gmplot
from haversine import haversine, Unit
import csv
import cv2
import math

class Gps:
    def __init__(self):
        self.key = ""
        self.destination = "24.179035,120.651736"
        self.mode = "walking"
        self.traffic_model = "optimistic"
        self.optimize_waypoints = False
        self.way_point = []
        self.robot_location = [0,0]
        self.gps_state = False
        self.navigation_state = False
        self.distance = 100
        self.angle = 100
        
    def directions(self,robot_location):
        gmaps=googlemaps.Client(self.key)
        routes = gmaps.directions(self.robot_location, self.destination, self.mode, self.optimize_waypoints)
        polyline = (routes[0]["overview_polyline"]["points"]).encode()
        lnglat_point = decode_polyline(polyline, 5)
        print("Path point number: {}".format(len(lnglat_point)))
        #從這裡開始是畫出當次Google map的路徑點地圖
        gmap = gmplot.GoogleMapPlotter(lnglat_point[0][1],lnglat_point[0][0] , 22 ,apikey = self.key)
        lat_point = []
        lng_point = []
        for i in range(len(lnglat_point)):
            gmap.marker(lnglat_point[i][1] , lnglat_point[i][0] , label=str(i) ,color='red')
            lat_point.append(lnglat_point[i][1])
            lng_point.append(lnglat_point[i][0])
        gmap.plot(lat_point , lng_point, 'cornflowerblue', edge_width = 3.0)
        gmap.draw('./map.html')
        self.way_point = list(zip(lat_point,lng_point))
        
        return self.way_point
        
#這個Method需要把Ardiuno輸入進來的GPS讀數進行過濾，以防讀取延遲造成數字錯位
#經緯度寫入的字串會是"xx.xxxxxxx,xx.xxxxxx"，以","作為分隔符號。
#只有在符合格式時才會把經緯度寫入到class的attribute裡，否則會回傳false的狀態
    def get_data(self,data):
        if len(data) <= 24 and len(data) >= 20:
            GPS_data = data
            GPS_data2 = GPS_data.partition(',')
            if len(GPS_data2) == 3:
                print("GPS:",data)
                lat = int(GPS_data2[0])*(1E-7)
                lng = int(GPS_data2[2])*(1E-7)
                self.robot_location = [lat,lng]
                #每次執行這個Function就把目前的坐標寫入的GPS_path.csv
                f = open('./GPS_path.csv',mode='a',newline='')
                writer = csv.writer(f)
                writer.writerow(self.robot_location)
                f.close()
                self.gps_state = True
            else:
                self.gps_state = False
        else:
            self.gps_state = False
            
        return  self.gps_state, self.robot_location
    
    def distance_to_point(self,robot_location):
        if self.way_point != []:
            next_point = (self.way_point[0][0],self.way_point[0][1])
            self.distance = haversine(robot_location , next_point , unit=Unit.METERS)
            #如果目前經緯度與現在的目標點經緯度的差小於1m 則刪去現在的目標點
            if self.distance<10 and len(self.way_point)>1:
                self.way_point.remove(self.way_point[0])
            #計算下一點導航點與目前位置的方位角    
            dx = self.way_point[0][0]-self.robot_location[0]
            dy = self.way_point[0][1]-self.robot_location[1]
            self.angle = (180/math.pi)*math.atan2(dy,dx)
            self.navigation_state = False
        else:
            self.navigation_state = True
        
        return self.navigation_state, self.distance, self.angle
    
    def data_img_draw(self,data_img,delta_angle,magnetic):
        cv2.putText(data_img, str("Distance to next point: {:.2f}m".format(self.distance)), (20 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("Angle to next point: {:.2f}".format(self.angle)), (20 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("GPS of self location: {0:.7f},{1:.7f}".format(self.robot_location[0],self.robot_location[1])), (20 , 160), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("GPS of next point: {0:.7f},{1:.7f}".format(self.way_point[0][0] , self.way_point[0][1])), (20 , 140), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("delta angle: {0:.1f}".format(delta_angle)), (20 , 120), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("Face angle: {0:.2f}".format(magnetic)), (20 , 100), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
        
        return data_img
        
if __name__ == '__main__':#function test
    GPS = Gps()
    print(GPS.gps_state)
    gps_ret, lat_lng = GPS.get_data("241796933, 1206474330")
    print(gps_ret, lat_lng)
    if gps_ret == True:
        print(GPS.key,GPS.robot_location,GPS.destination,GPS.mode,GPS.traffic_model)
        way_point = GPS.directions(lat_lng)
        print(way_point)