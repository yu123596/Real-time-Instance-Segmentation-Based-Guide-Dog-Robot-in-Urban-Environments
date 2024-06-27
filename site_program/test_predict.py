# -*- coding: utf-8 -*-
"""
Created on Tue Apr 25 15:20:41 2023

@author: 309-1-RTX3060
"""
import numpy as np
import urllib
import time
import cv2
from yolact_edge.inference import YOLACTEdgeInference
import os 
import pyzed.sl as sl

weights = "./weights/11_16_roadwithcoco_resnet50_55_25000.pth"
config = 'road_resnet50_config'
dataset = 'road_dataset'
#calib_images = "C:/Users/309-1-RTX3060/Downloads/MTRdataset/image/"
calib_images = "C:/Users/309-1-RTX3060/Downloads/MTRdataset/image/00002610.jpg"
#images_num = os.listdir(calib_images)
#images_path = len(images_num)
config_ovr = {
    'use_fast_nms': True,  # Does not work with regular nms
    'mask_proto_debug': False
}
model_inference = YOLACTEdgeInference(
    weights, config, dataset, calib_images, config_ovr)
#model setting============================================
img = None


print("Benchmarking performance...")
start = time.time()
#samples = images_path

img = cv2.imread(calib_images)  
p = model_inference.predict(img, True)
if p != None:    
    cv2.imshow('i',p["img"])    
cv2.waitKey(0) & 0xFF == ord('q')
cv2.destroyAllWindows()
#print(f"Average {1 / ( (time.time() - start) / samples )} FPS")

#=======
'''
import pyzed.sl as sl
import math
import numpy as np
import sys
import cv2

zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_fps = 30 # Set fps at 30
init_params.depth_minimum_distance = 200

cam = sl.Camera()
if not cam.is_opened():
    print("Opening ZED Camera...")
status = cam.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

runtime = sl.RuntimeParameters()
Left_image = sl.Mat()
Depth_view = sl.Mat()
err = cam.grab(runtime)
 
while err == sl.ERROR_CODE.SUCCESS:
    err = cam.grab(runtime)
    cam.retrieve_image(Left_image, sl.VIEW.LEFT)
    cam.retrieve_image(Depth_view, sl.VIEW.DEPTH)
    #image = cv2.cvtColor(mat.get_data(), cv2.COLOR_RGBA2RGB)
    
    cv2.imshow("Left", Left_image.get_data())
    cv2.imshow("Depth", Depth_view.get_data())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()

cam.close()
print("\nFINISH")
'''