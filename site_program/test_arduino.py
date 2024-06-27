# -*- coding: utf-8 -*-
"""
Created on Thu Jul  6 01:33:05 2023

@author: 309-1-RTX3060
"""

import serial
import time
COM_PORT = 'COM3'  # 根據連結的Arduino的通訊埠修改設定
BAUD_RATES = 115200
Ser = serial.Serial(COM_PORT, BAUD_RATES)

try:
    #buffer = list()
    
    while True:
        while Ser.in_waiting:
            data_raw = Ser.read().decode('utf-8')
            #buffer.append(data_raw)
            Ser.in_waiting
            if data_raw == 'R':
                print('收到的資料：', data_raw)
                #buffer.clear()
            if data_raw == 'O':
                print('收到的資料：', data_raw)
                #.clear()
            if data_raw == 'Q':
                print('收到的資料：', data_raw)
                #buffer.clear()

except KeyboardInterrupt:
    Ser.close()    # 清除序列通訊物件
    print('關閉程式')