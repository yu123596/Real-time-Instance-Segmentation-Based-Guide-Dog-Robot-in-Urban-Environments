# -*- coding: utf-8 -*-
"""
Created on Fri Jun  7 04:38:23 2024

@author: 309-1-RTX3060
"""
import numpy as np  
import time
import cv2

class Model_Info:
    def __init__(self):
        self.weights = "./weights/b37m38road_resnet50_214_96000.pth"
        self.config = 'road_resnet50_config'
        self.dataset = 'road_dataset'
        self.calib_images = "./data/coco/calib_images"
        self.config_ovr = {'use_fast_nms': True,'mask_proto_debug': False}
        
# =============================================================================
# config_ovr = {
# 'torch2trt_backbone_int8': True,
# 'torch2trt_fpn': True,
# 'torch2trt_protonet_int8': True,
# 'torch2trt_prediction_module': True,
# 'use_fast_nms': True,  # Does not work with regular nms
# 'mask_proto_debug': False
# }
# =============================================================================  
class EnvironmentDetection:
    def __init__(self):
        self.ID = 3
        self.class_num = 5
        self.arduino_msg = str(0)
        self.far_mask_state_change = 0
        self.mid_mask_state_change = 0
        self.close_mask_state_change = 0
        
        self.farline_class = 0
        self.mid_class = 0
        self.close_class = 0
        self.old_farline_class = np.zeros(self.class_num)
        self.old_mid_class_state = np.zeros(self.class_num)
        self.old_close_class = np.zeros(self.class_num)
        
        self.cross_state = False
        self.roadout_state = False
        self.lose_road_t
    
    def farline_detect(self,far_mask_precent):#遠探測線探知
        #如果斑馬線和人行道在遠探測線上的比例一樣或差距少於20% 則視為斑馬線
        if far_mask_precent[0]-far_mask_precent[1]==0 or far_mask_precent[0]-far_mask_precent[1]<=0.2:
             far_mask_precent[1] = far_mask_precent[1]*2
        #1)若本次偵測Mask中最高分的class與上次紀錄的class不同 則計數器+1      
        if self.old_farline_class != far_mask_precent.index(max(far_mask_precent)):
            #2)若計數器大於5(即狀態改變為偵測class持續3幀時) 狀態為當時的class
            self.far_mask_state_change = self.far_mask_state_change+1
            if self.far_mask_state_change > 5:
                #class判斷線的狀態改變  
                    self.farline_class = far_mask_precent.index(max(far_mask_precent))
                    self.far_mask_state_change = 0
        
        return self.farline_class
    
    def midline_detect(self,mid_mask_precent):
        if self.old_mid_class_state != mid_mask_precent.index(max(mid_mask_precent)):
            self.mid_mask_state_change = self.mid_mask_state_change+1
            if self.mid_mask_state_change > 5:
                    self.mid_class = mid_mask_precent.index(max(mid_mask_precent))
                    self.mid_mask_state_change = 0
                    
        return self.midline_class        
    
    def closeline_detect(self,close_mask_precent):
        if self.old_close_class != close_mask_precent.index(max(close_mask_precent)):
            self.close_mask_state_change = self.close_mask_state_change+1
            if self.close_mask_state_change > 5:
                    self.close_class = close_mask_precent.index(max(close_mask_precent))
                    self.close_mask_state_change = 0
                    
        return self.closeline_class  
    
    #斑馬線和馬路探測功能
    def crosswalk_detect(self):
        #當畫面中部和底部的Mask狀態分別為人行道和斑馬線時等下並通知前方為斑馬線 
        #而斑馬線到None或人行道時不能停下來 
        if  self.old_farline_class != 1 and self.farline_class == 1 and self.close_class == 0:
            if self.close_class == 0 or self.close_class == 4:
                self.arduino_msg= f'{self.ID}_190_0\n'
                print('Ready to Crosswalk')
                self.qua_state = self.qua_state+1
                self.cross_state = True
        elif self.farline_class == 1 and self.close_class != 0 and self.cross_state == 1:
            self.cross_state = False
        if self.qua_state >0 and self.qua_state <=5 and self.cross_state == 1:
            self.qua_state = self.qua_state+1
            self.arduino_msg= f'{self.ID}_190_0\n'
        elif self.qua_state>5:
            self.qua_state = 0
        #更新舊探測線的狀態為當前的狀態
        self.old_farline_class = self.farline_class
        
        return self.arduino_msg
    
    def road_detect(self):
        if self.roadout_state == True:#
            if self.old_mid_class_state == 3  and self.mid_class == 3 and self.close_class == 0:#
                self.arduino_msg=f'{self.ID}_140_0\n'
                print('Going back 2 time')
            elif self.old_mid_class_state == 3  and self.mid_class == 0 and self.close_class == 0:
                self.roadout_state = False
                self.lose_road_t = 0
                print('roadout_state = 0')
            
        if self.old_mid_class_state != 3  and self.mid_class == 3 and self.close_class == 0 and self.roadout_state == False and self.lose_road_t == 0:#1秒內辨識結果超過10幀的分數  roadout_state為1
            self.lose_road_s = time.time()
            self.lose_road_t = 1
            print('lose_pre_t = 1')
        elif self.mid_class_state == 3 and self.close_class == 0 and self.roadout_state == False and self.lose_road_t != 0:
            new_lose_road_s = time.time()
            if new_lose_road_s - self.lose_road_s <1:
                self.lose_road_t = self.lose_road_t+1
            else:
                self.lose_road_t = 0
            print('lose_road_t + 1')
            if self.lose_road_t > 10:
                self.roadout_state = True
                self.arduino_msg=f'{self.ID}_130_0\n'
                print('roadout_state == 1')
        #更新舊探測線的狀態為當前的狀態
        self.old_mid_class_state = self.mid_class
        self.old_close_class = self.close_class
                
        return self.arduino_msg
    
    def detecting_environmental(self,arduino_msg):
        #愈優先的判斷放在愈後面
        self.arduino_msg = arduino_msg
        self.arduino_msg = self.crosswalk_detect()
        self.arduino_msg = self.road_detect()
        
        return self.arduino_msg
    
    def data_img_draw(self,data_img,far_mask_precent,mid_mask_precent,close_mask_precent):
        cv2.putText(data_img, str("Class of Far line: {:.0f}".format(self.farline_class)), (350 , 20), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("Class of Mid line: {:.0f}".format(self.mid_class)), (350 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("Class of Close line: {:.0f}".format(self.close_class_state)), (350 , 100), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)   
        cv2.putText(data_img, str("{0:.1f} | {1:.1f} | {2:.1f} | {3:.1f} | {4:.1f}".format(far_mask_precent[0],far_mask_precent[1],far_mask_precent[2],far_mask_precent[3],far_mask_precent[4])), (350 , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("{0:.1f} | {1:.1f} | {2:.1f} | {3:.1f} | {4:.1f}".format(mid_mask_precent[0],mid_mask_precent[1],mid_mask_precent[2],mid_mask_precent[3],mid_mask_precent[4])), (350 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("{0:.1f} | {1:.1f} | {2:.1f} | {3:.1f} | {4:.1f}".format(close_mask_precent[0],close_mask_precent[1],close_mask_precent[2],close_mask_precent[3],close_mask_precent[4])), (350 , 120), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        #cv2.putText(data_img, str("fix_angle_state: {0} {1}".format(fix_angle_state)), (350 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
        
        
        return data_img
    
if __name__ == '__main__': #function test
    pass
