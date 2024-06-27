# -*- coding: utf-8 -*-
"""
Created on Fri Jun  7 08:01:05 2024

@author: 309-1-RTX3060
"""
import numpy as np
import time
import cv2

class Scoring:
    def __init__(self):
        self.area_num=7 #把畫面切成七份
        self.area_w = int(1280/self.area_num)#每份畫面的寬度和長度
        self.mask_h = int(594)
        self.depth_h = int(550)
        self.area_start_y = 0
        self.max_distance = 250
        #gps評分參數
        self.all_area_angle  = list(np.zeros(self.area_num))
        self.gps_score = list(np.zeros(self.area_num))
        self.gps_rank = list(np.zeros(self.area_num))
        self.delta_angle = 0
        self.fix_angle_state = 0
        #depth評分參數
        self.depth_area_score = list(np.zeros(self.area_num))
        self.depth_area_rank = list(np.zeros(self.area_num))
        #Mask評分參數
        self.mask_area_score = list(np.zeros(self.area_num))
        self.mask_area_rank = list(np.zeros(self.area_num))
        self.road_classes_num = 5
        self.far_mask_precent = list(np.zeros(self.road_classes_num))
        self.mid_mask_precent = list(np.zeros(self.road_classes_num))
        self.close_mask_precent = list(np.zeros(self.road_classes_num))
        #最終評分參數
        self.average_score = list(np.zeros(self.area_num))
        self.average_rank = list(np.zeros(self.area_num))
        self.highest_score_area = 0
        
    def area_angle(self,magnetic):
        #感測器的磁力計數值範圍是0 ~ 180 和 0 ~ -180 
        #所以要寫一個超過180度時把角度反轉成由-180到0的程式
        for g in range(self.area_num): #g是畫面切片的第幾份
            if g < 3:
                self.gps_score[g] = magnetic+17*-(3-g)
            elif g >3 :
                self.gps_score[g] = magnetic+17*-(3-g)
            elif g ==3 :
                self.gps_score[g] = magnetic
        for g in range(self.area_num):
            if abs(self.gps_score[g])>180 and self.gps_score[g] >-129:
                self.gps_score[g] = -180-(180-self.gps_score[g])
            elif abs(self.gps_score[g])>180 and self.gps_score[g] <-129:
                self.gps_score[g] = 180-(abs(180+self.gps_score[g]))
        self.all_area_angle = list(np.round(self.gps_score))
        
        return self.all_area_angle 
    
    def gps_scoreing(self,angle):
        #計算各個方向與坐標點的角度差
        #angle是坐標點與自身的相對方位角
        for g in range(self.area_num):
            if angle >=0 and self.gps_score[g] >=0:
                fix_angle=abs(angle-self.gps_score[g])
                self.gps_score[g]=1-fix_angle/180
                if g == 3:
                    self.delta_angle = fix_angle
                    if self.all_area_angle[g] > angle :
                        fix_angle_state=1
                    else:
                        fix_angle_state=2
            elif angle >=0 and self.gps_score[g] <=0:
                if self.gps_score[g] > -(180-angle) :
                    fix_angle=abs(angle-self.gps_score[g])
                    self.gps_score[g]=1-fix_angle/180
                    if g == 3:
                        self.delta_angle = fix_angle
                        fix_angle_state=2
                else:
                    fix_angle=abs(180+self.gps_score[g]+180-angle)
                    self.gps_score[g]=1-fix_angle/180
                    if g == 3:
                        self.delta_angle = fix_angle
                        fix_angle_state=1
            elif angle <=0 and self.gps_score[g] <=0:
                fix_angle=abs(angle-self.gps_score[g])
                self.gps_score[g]=1-fix_angle/180
                if g == 3:
                    self.delta_angle = fix_angle
                    if self.all_area_angle[g] > angle :
                        fix_angle_state=1
                    else:
                        fix_angle_state=2
            elif angle <=0 and self.gps_score[g] >=0:
                if self.all_area_angle[g] > (180+angle) :
                    fix_angle=abs(180+angle+(180-self.gps_score[g]))
                    self.gps_score[g]=1-fix_angle/180
                    if g == 3:
                        self.delta_angle = fix_angle
                        fix_angle_state=2
                else:
                    fix_angle=abs(-angle+self.gps_score[g])
                    self.gps_score[g]=1-fix_angle/180
                    if g == 3:
                        self.delta_angle = fix_angle
                        fix_angle_state=1
        self.gps_score = np.array(self.gps_score)
        self.gps_rank = np.argsort(self.gps_score)
        self.gps_rank = np.lexsort((self.gps_score,self.gps_rank))
        
        return self.gps_score,self.gps_rank,fix_angle_state,self.delta_angle
        
    def depth_scoring(self,depth_GRAY):
        self.depth_area_score = list(np.zeros(self.area_num))
        self.depth_area_rank = list(np.zeros(self.area_num))
        for a in range(self.area_num):#每一個Mask和depth_map分成7等分
            area_start_x = self.area_w*(a)+a
            area_end_x = self.area_w*(a+1)+a
            depth_area = depth_GRAY[self.area_start_y:self.depth_h,area_start_x:area_end_x]
            if np.max(depth_area) > self.max_distance: #當depth map像素距離到閥值(過近) 分數為-10
                self.depth_area_score[a] = -10
            else :
                self.depth_area_score[a] = self.depth_area_score[a]+(1-np.mean(depth_area)/255) #前方愈空曠愈高分
        self.depth_area_score = np.array(self.depth_area_score)
        self.depth_area_rank = np.argsort(self.depth_area_score)
        self.depth_area_rank = np.lexsort((self.depth_area_score,self.depth_area_rank))
        self.depth_area_rank = np.where(self.depth_area_score>0,self.depth_area_rank,-5)
            
        return self.depth_area_score,self.depth_area_rank
    
    def mask_precent(self,p_class,p_mask):
        self.mask_area_score = list(np.zeros(self.area_num))
        self.mask_area_rank = list(np.zeros(self.area_num))
        self.far_mask_precent = list(np.zeros(self.road_classes_num))
        self.mid_mask_precent = list(np.zeros(self.road_classes_num))
        self.close_mask_precent = list(np.zeros(self.road_classes_num))
        for m in range(len(p_class)): #m是辨識出幾個mask
            area_start_x = 0
            area_end_x = 0
            if len(p_class) == 1:#畫面只有一種或多種Mask的判斷邏輯
                tensor_mask = p_mask
                tensor_mask = np.uint8((tensor_mask).cpu()) #把模型輸出的mask從GPU放到CPU 再變成8位元圖
                class_num = int(p_class)
                #far_class_mask = tensor_mask[188, 223:447]
                #close_class_mask = tensor_mask[360, 223:447]
                far_class_mask = tensor_mask[160, 365:921]
                mid_class_mask = tensor_mask[220, 365:921]
                close_class_mask = tensor_mask[370, 365:921]
                self.far_mask_precent[class_num] = np.mean(far_class_mask)
                self.mid_mask_precent[class_num] = np.mean(mid_class_mask)
                self.close_mask_precent[class_num] = np.mean(close_class_mask)
            else:
                tensor_mask = p_mask[m]
                tensor_mask = np.uint8((tensor_mask).cpu())
                class_num = int(p_class[m])
                #far_class_mask = tensor_mask[188, 223:447]
                #close_class_mask = tensor_mask[360, 223:447]
                far_class_mask = tensor_mask[160, 365:921]
                mid_class_mask = tensor_mask[220, 365:921]
                close_class_mask = tensor_mask[370, 365:921]
                self.far_mask_precent[class_num] = self.far_mask_precent[class_num]+np.mean(far_class_mask)
                self.mid_mask_precent[class_num] = self.mid_mask_precent[class_num]+np.mean(mid_class_mask)
                self.close_mask_precent[class_num] = self.close_mask_precent[class_num]+np.mean(close_class_mask)
            for a in range(self.area_num):#每一個Mask和depth_map分成7等分
                area_start_x = self.area_w*(a)+a
                area_end_x = self.area_w*(a+1)+a
                mask_area = tensor_mask[self.area_start_y:self.mask_h,area_start_x:area_end_x]
                #計算Mask每個等分的平均值
                if class_num != 3: #除了3號class(Road)減分 其他的Mask均為加分
                    self.mask_area_score[a] = self.mask_area_score[a]+np.average(mask_area)
                else:
                    self.mask_area_score[a] = self.mask_area_score[a]-np.average(mask_area)    
        self.far_mask_precent[4] = 1-np.sum(self.far_mask_precent)
        self.mid_mask_precent[4] = 1-np.sum(self.mid_mask_precent)  
        self.close_mask_precent[4] = 1-np.sum(self.close_mask_precent)
        
        return self.far_mask_precent,self.mid_mask_precent,self.close_mask_precent
    
    def mask_scoring(self):
        for a in range(self.area_num):
            if self.mask_area_score[a] <= 0:
                self.mask_area_score[a] = -10
        self.mask_area_score = np.array(self.mask_area_score)
        self.mask_area_rank = np.argsort(self.mask_area_score)
        self.mask_area_rank = np.lexsort((self.mask_area_score,self.mask_area_rank))
        self.mask_area_rank = np.where(self.depth_area_rank>=0,self.mask_area_rank,-10)
        self.mask_area_rank = np.where(self.mask_area_score>0,self.mask_area_rank,-10)

        return self.mask_area_score,self.mask_area_rank
    
    def final_scoring(self,p_state):
        if p_state == True:
            self.average_score = self.gps_rank+self.depth_area_rank+self.mask_area_rank
        else:
            self.average_score = self.gps_rank+self.depth_area_rank
        self.average_rank = np.argsort(self.average_score)
        self.average_rank = np.lexsort((self.average_score,self.average_rank))
        
        return self.average_score, self.average_rank
    
    def final_decision(self):
        self.highest_score_area = np.argmax(self.average_rank)
        #沒有把depth_area_rank讀進來
        if self.depth_area_rank[3]<0:#如果正前方有障礙物
            if self.highest_score_area == 2 :#如果最高分的方向是偏左
                self.highest_score_area = self.highest_score_area-1#則最終決策再往左邊一格
            elif self.highest_score_area ==4:#如果最高分的方向是偏右
                self.highest_score_area = self.highest_score_area+1#則最終決策再往右邊一格
        elif self.depth_area_rank[2]<0 and self.highest_score_area ==3:
            #如果偏左的方向有障礙物  最高分的方向是中間
            self.highest_score_area = self.highest_score_area+1#則最終決策往左邊一格
        elif self.depth_area_rank[4]<0 and self.highest_score_area ==3:
            #如果偏右的方向有障礙物  最高分的方向是中間
            self.highest_score_area = self.highest_score_area-1#則最終決策往右邊一格
                
        if self.highest_score_area == 3:
            if self.average_rank[2] == -5 or self.average_rank[2] < self.average_rank[4]:
                if self.average_score[4] > 0 and self.average_score[5] > 0 and self.average_score[6] > 0: #left side to close or no detect
                    self.highest_score_area = 4
            if self.average_rank[4] == -5 or self.average_rank[4] < self.average_rank[2]:
                if self.average_score[2] >0  and self.average_score[1] > 0 and self.average_score[0] > 0: #right side to close or no detect
                    self.highest_score_area = 2
        final_decision = self.highest_score_area*20
        
        return final_decision
    
    def p_img_draw(self,p_state,p_img,Depth_GRAY):
        cv2.circle(p_img,((self.area_w*(self.highest_score_area+1)-48),170),5,(0,255,0),3)
        cv2.line(p_img, (365,160), (921,160), (0,0,255), 2)
        cv2.line(p_img, (365,220), (921,220), (0,0,255), 2)
        cv2.line(p_img, (365,370), (921,370), (0,0,255), 2)
        
        for score_num in range(self.area_num):
            cv2.line(p_img, (self.area_w*(score_num+1)+score_num,0), (self.area_w*(score_num+1)+score_num,720), (0, 255, 255), 1)
            cv2.line(Depth_GRAY, (self.area_w*(score_num+1)+score_num,0), (self.area_w*(score_num+1)+score_num,720), (255), 1)
            cv2.putText(p_img, str("{:.2f}".format(self.gps_score[score_num])), ((60+self.area_w*(score_num)) , 250), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(p_img, str("{}".format(self.gps_rank[score_num])), ((60+self.area_w*(score_num)) , 270), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(p_img, str("{:.2f}".format(self.depth_area_score[score_num])), ((60+self.area_w*(score_num)) , 290), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(p_img, str("{}".format(self.depth_area_rank[score_num])), ((60+self.area_w*(score_num)) , 310), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
            if p_state == True:
                cv2.putText(p_img, str("{:.2f}".format(self.mask_area_score[score_num])), ((60+self.area_w*(score_num)) , 330), cv2.FONT_HERSHEY_DUPLEX,0.5, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(p_img, str("{}".format(self.mask_area_rank[score_num])), ((60+self.area_w*(score_num)) , 350), cv2.FONT_HERSHEY_DUPLEX,0.5, (255, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(p_img, str("{} ({})".format(self.average_rank[score_num],self.average_score[score_num])), ((60+self.area_w*(score_num)) , 390), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 0), 1, cv2.LINE_AA)
            
            
        return p_img,Depth_GRAY

class RobotDecision:
    def __init__(self):
        self.class_state = 4
        self.roadout_state = False
        self.Depth_back_state = False

        self.arduino_msg = str(0)
        self.search_object = []
        self.send_time=0
        self.send_state = False

        self.close_object_loc = 0
        self.TOF_state = False
        self.cross_state = False
        self.Motor = 13
        
        self.ID = 3
        self.turn_state = False
        self.old_pose_state=[]
        #姿態變數
        self.qua_state = 0 #陀螺儀讀數超過閾值連續幀數
        self.pose_state = 0 #陀螺儀狀態 1代表上仰 -1代表下俯
        #depth_detect變數
        self.new_blocked_time = 0
        self.turn_back_state = 0
        self.blocked_time = 0
        self.total_blocked_time = 0
        self.lose_pre_t = 0
        self.lose_pre_state = False
        self.lose_road_s = 0
        self.lose_road_t = 0
    
    
    #depth_detect用作處理辦識評分總和為負值時的狀態，
    #共0 1 2三個狀態 0為從未觸發 1為首次探測到障礙物(原地停止) 2為前有障礙物超過2秒(後退)
    def depth_detect_block(self,depth_area_score):
        if self.turn_back_state == 2:#因深度感測到曾被阻隔超過4秒 
            if np.sum(depth_area_score)>-40:#執行後退直至至少3個方向沒到閥值狀態
                self.turn_back_state = 0
                self.blocked_time = 0
                self.new_blocked_time = 0
                self.total_blocked_time = 0
            else:
                self.arduino_msg=f'{self.ID}_140_0\n'
                print('Going back')
                
        return self.arduino_msg
    
    def depth_detect_count(self,depth_area_score):
        #深度圖中7個area全部都到閥值 回頭狀態為0 執行停下等待2秒(等移動障礙物自行離開)
        if np.sum(depth_area_score)==-70 and self.turn_back_state == 0: 
            self.blocked_time = time.time()
            self.arduino_msg=f'{self.ID}_130_0\n'
            print('Depth noway to go')
            self.turn_back_state = self.turn_back_state+1
        #深度圖中7個area全部都到閥值 回頭狀態為1 
        elif np.sum(depth_area_score)==-70 and self.turn_back_state==1:
            new_blocked_time = time.time()
            self.total_blocked_time = new_blocked_time-self.blocked_time
        #深度圖中7個area全部都到閥值  阻擋時間大於2秒 回頭狀態+1
        elif np.sum(depth_area_score)==-70 and self.turn_back_state==1 and self.total_blocked_time > 2:
            self.turn_back_state = self.turn_back_state+1
            self.blocked_time = time.time()
        #深度圖中7個area全部都到閥值 回頭狀態為2
        elif np.sum(depth_area_score)==-70 and self.turn_back_state == 2: 
            new_blocked_time = time.time()
            self.total_blocked_time = new_blocked_time-self.blocked_time
            
        return self.arduino_msg
    
    def depth_detect(self,depth_area_score):
        self.arduino_msg = self.depth_detect_block(depth_area_score)
        self.arduino_msg = self.depth_detect_count(depth_area_score)
    
        return self.arduino_msg
    
    #lose_mask用作處理辦識評分總和為負值時的狀態，即視野超過一半是馬路或無辨識結果
    def lose_mask_back(self,mask_area_rank,mask_area_score):
        if self.lose_pre_state == True:#因1秒內辨識評分總和小於0超過10幀 且目前辨識評分總和仍然小於0
            print(str(np.sum(mask_area_rank)))
            if np.sum(mask_area_score)<-10:#執行後退直至辨識評分總和大於-30(畫面上有可行走區域3/7
                self.arduino_msg=f'{self.ID}_140_0\n'
                print('Going back 2 time')
            else:
                self.lose_pre_state = False
                self.lose_pre_t = 0
                print('lose_pre_state = 0')
                
        return self.arduino_msg
    
    def lose_mask_stop(self,mask_area_rank,mask_area_score):
        if np.sum(mask_area_rank)<=-30 and self.lose_pre_t == 0:#1秒內辨識結果超過10幀的分數相加小於-30時  lose_pre_state為1
            self.lose_pre_s = time.time()
            self.lose_pre_t = 1
            #print('lose_pre_t = 1')
        elif np.sum(mask_area_rank)<=-30 and self.lose_pre_t != 0:
            new_lose_pre_s = time.time()
            if new_lose_pre_s - self.lose_pre_s <1:
                self.lose_pre_t = self.lose_pre_t+1
            else:
                self.lose_pre_t = 0
            #print('lose_pre_t + 1')
            if self.lose_pre_t > 10:
                self.lose_pre_state = True
                self.arduino_msg=f'{self.ID}_130_0\n'
                print('lose_pre_state == 1')
        
        return self.arduino_msg
    
    def lose_mask_detect(self,mask_area_rank,mask_area_score):
        self.arduino_msg = self.lose_mask_back(mask_area_rank,mask_area_score)
        self.arduino_msg = self.lose_mask_stop(mask_area_rank,mask_area_score)

        return self.arduino_msg
    
    def vision_decision(self,arduino_msg,depth_area_score,mask_area_rank,mask_area_score):
        self.arduino_msg = arduino_msg
        self.arduino_msg = self.depth_detect(depth_area_score)
        self.rduino_msg = self.lose_mask_detect(mask_area_rank,mask_area_score)
        
        return self.arduino_msg
    
    
    
    def face2loca(self,delta_angle,fix_angle_state):
        if delta_angle>90:
            self.turn_state = 1
        if self.turn_state == 1 and delta_angle>5:
            if fix_angle_state == 1:
                self.arduino_msg=f'{self.ID}_150_0\n'
                print('turning left')
            if fix_angle_state == 2:
                self.arduino_msg=f'{self.ID}_160_0\n'
                print('turning right')
        elif self.turn_state == 1 and delta_angle<5:
             self.turn_state = 0
             
        return self.arduino_msg
    
    #每次陀螺儀讀數超過閾值 錄得連續4幀才確認姿態的改變
    def RobotPosition(self,fix_quaternion):
        if fix_quaternion[0] > 10 and self.pose_state == 0:
            print('Going up')
            self.arduino_msg = f'{self.ID}_180_0\n'
            self.qua_state = self.qua_state+1
            if self.qua_state > 3:
                self.pose_state = 1
                self.qua_state = 0
        elif fix_quaternion[0] < 10 and self.pose_state == 1:
            self.pose_state = 0
        elif fix_quaternion[0] < -10 and self.pose_state == 0:
            print('Going down')
            self.arduino_msg = f'{self.ID}_170_0\n'
            if self.qua_state > 3:
                self.pose_state = -1
                self.qua_state = 0
        elif fix_quaternion[0] > -10 and self.pose_state == -1:
            self.pose_state = 0
            #陀螺儀感測動作
            
        return self.arduino_msg

    def sensor_decision(self,fix_quaternion,delta_angle,fix_angle_state):
        self.arduino_msg = self.face2loca(delta_angle,fix_angle_state)
        self.arduino_msg = self.RobotPosition(fix_quaternion)
        
        return self.arduino_msg

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
        self.old_farline_class = list(np.zeros(self.class_num))
        self.old_mid_class_state = list(np.zeros(self.class_num))
        self.old_close_class = list(np.zeros(self.class_num))
        
        self.cross_state = False
        self.roadout_state = False
        self.lose_road_t = 0
        self.qua_state = 0
    
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
                    
        return self.mid_class        
    
    def closeline_detect(self,close_mask_precent):
        if self.old_close_class != close_mask_precent.index(max(close_mask_precent)):
            self.close_mask_state_change = self.close_mask_state_change+1
            if self.close_mask_state_change > 5:
                    self.close_class = close_mask_precent.index(max(close_mask_precent))
                    self.close_mask_state_change = 0
                    
        return self.close_class
    
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
        elif self.mid_class == 3 and self.close_class == 0 and self.roadout_state == False and self.lose_road_t != 0:
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
        cv2.putText(data_img, str("Class of Close line: {:.0f}".format(self.close_class)), (350 , 100), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)   
        cv2.putText(data_img, str("{0:.1f} | {1:.1f} | {2:.1f} | {3:.1f} | {4:.1f}".format(far_mask_precent[0],far_mask_precent[1],far_mask_precent[2],far_mask_precent[3],far_mask_precent[4])), (350 , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("{0:.1f} | {1:.1f} | {2:.1f} | {3:.1f} | {4:.1f}".format(mid_mask_precent[0],mid_mask_precent[1],mid_mask_precent[2],mid_mask_precent[3],mid_mask_precent[4])), (350 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(data_img, str("{0:.1f} | {1:.1f} | {2:.1f} | {3:.1f} | {4:.1f}".format(close_mask_precent[0],close_mask_precent[1],close_mask_precent[2],close_mask_precent[3],close_mask_precent[4])), (350 , 120), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        #cv2.putText(data_img, str("fix_angle_state: {0} {1}".format(fix_angle_state)), (350 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
        
        
        return data_img

if __name__ == "__main__":
    pass
