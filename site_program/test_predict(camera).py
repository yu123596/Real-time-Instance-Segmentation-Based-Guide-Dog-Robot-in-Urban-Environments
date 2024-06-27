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
calib_images = "C:/Users/309-1-RTX3060/Downloads/MTRdataset/image/"
images_num = os.listdir(calib_images)
images_path = len(images_num)
config_ovr = {
    'use_fast_nms': True,  # Does not work with regular nms
    'mask_proto_debug': False
}
model_inference = YOLACTEdgeInference(
    weights, config, dataset, calib_images, config_ovr)
#model setting============================================

init_params = sl.InitParameters()
init_params.camera_fps = 30 # Set fps at 30

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
#camera setting===========================================
#area_w = int(Left_RGB.shape[1]/7)
#area_h = int(Left_RGB.shape[1])
area_w = int(1280/7)
area_h = int(720)
area_start_y = 0
max_distance = 230

while err == sl.ERROR_CODE.SUCCESS:
    err = cam.grab(runtime)
    cam.retrieve_image(Left_image, sl.VIEW.LEFT)
    cam.retrieve_image(Depth_view, sl.VIEW.DEPTH)
    Left_RGB = cv2.cvtColor(Left_image.get_data(), cv2.COLOR_RGBA2RGB)
    Depth_GRAY = cv2.cvtColor(Depth_view.get_data(), cv2.COLOR_RGBA2GRAY)
    cv2.imshow("Depth", Depth_GRAY)
    p = model_inference.predict(Left_RGB, True)
    if p != None:
        Mask_area_score = [0,0,0,0,0,0,0]
        Depth_area_score = [0,0,0,0,0,0,0]
        average_score = [0,0,0,0,0,0,0]
        for mask_num in range(len(p["class"])):
            area_start_x = 0
            area_end_x = 0
            if len(p["class"]) == 1:
                tensor_mask = p["mask"]
                tensor_mask = np.uint8((tensor_mask).cpu())
                class_num = p["class"]
            else:
                tensor_mask = p["mask"][mask_num]
                tensor_mask = np.uint8((tensor_mask).cpu())
                class_num = p["class"][mask_num]
                
            for area_num in range(7):#每一個Mask和Depth_map分成7等分
                area_start_x = area_w*(area_num)+area_num
                area_end_x = area_w*(area_num+1)+area_num
                mask_area = tensor_mask[area_start_y:area_h,area_start_x:area_end_x]
                
                if class_num != 3: #除了3號class(Road)減分 其他的Mask均為加分
                    Mask_area_score[area_num] = Mask_area_score[area_num]+np.average(mask_area)
                else:
                    Mask_area_score[area_num] = Mask_area_score[area_num]-np.average(mask_area)
                #計算Mask每個等分的平均值
                
                Depth_area = Depth_GRAY[area_start_y:area_h,area_start_x:area_end_x]
                if np.max(Depth_area) >max_distance: #當depth map像素距離到閥值(過近) 分數為負無限
                    Depth_area_score[area_num] = -10
                else :
                    Depth_area_score[area_num] = Depth_area_score[area_num]+(1-(np.max(Depth_area)/255))
                #計算Depth_map不同區域的評分
            for avg_num in range(7):
                average_score[avg_num] = (Mask_area_score[avg_num]+Depth_area_score[avg_num])/2
             
        for score_num in range(7):
            cv2.line(p["img"], (area_w*(score_num+1)+score_num,0), (area_w*(score_num+1)+score_num,720), (0, 255, 255), 1)
            cv2.putText(p["img"], str("{:.2f}".format(Mask_area_score[score_num])), ((45+area_w*(score_num)) , 630), cv2.FONT_HERSHEY_DUPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(p["img"], str("{:.2f}".format(Depth_area_score[score_num])), ((45+area_w*(score_num)) , 650), cv2.FONT_HERSHEY_DUPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(p["img"], str("{:.2f}".format(average_score[score_num])), ((45+area_w*(score_num)) , 670), cv2.FONT_HERSHEY_DUPLEX,1, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("ZED", p["img"])
    else:
        cv2.imshow("ZED", Left_RGB)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()

cam.close()
print("\nFINISH")

#=======
'''
start = time.time()
samples = images_path
print("Benchmarking performance...")
for i_p in range (images_path):
    img = calib_images + images_num[i_p]
    img = cv2.imread(img)  
    p = model_inference.predict(img, True)
print(f"Average {1 / ( (time.time() - start) / samples )} FPS")

cv2.imshow("ZED", mat.get_data())
cv2.destroyAllWindows()

cam.close()
'''
'''
print("Running...")
init = sl.InitParameters()
cam = sl.Camera()
if not cam.is_opened():
    print("Opening ZED Camera...")
status = cam.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

runtime = sl.RuntimeParameters()
mat = sl.Mat()
err = cam.grab(runtime)
while err == sl.ERROR_CODE.SUCCESS:
    err = cam.grab(runtime)
    cam.retrieve_image(mat, sl.VIEW.LEFT)
    cv2.imshow("ZED", mat.get_data())

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()

cam.close()
print("\nFINISH")
'''