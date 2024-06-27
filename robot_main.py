# -*- coding: utf-8 -*-
"""
Created on Tue Apr 25 15:20:41 2023

@author: 309-1-RTX3060
"""
import numpy as np
import sys
import time
import cv2
from yolact_edge.inference import YOLACTEdgeInference
import os 
import pyzed.sl as sl
import serial
from site_program.gps_func import Gps
from site_program.Model_Info import Model_Info
from site_program.Scoring_func import Scoring,RobotDecision,EnvironmentDetection

class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()

    ##
    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.MagnetometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_mag.get_microseconds())
            if new_:
                self.t_mag = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.BarometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_baro.get_microseconds())
            if new_:
                self.t_baro = sensor.timestamp
            return new_
        
GPSser = serial.Serial('/dev/ttyACM0',115200,timeout=0.0001)
MotorSer = serial.Serial('/dev/ttyACM1',9600,timeout=0.001)
GPS = Gps()
model_info = Model_Info()
scoring = Scoring()
robot_decision = RobotDecision()
robot_environment_decision = EnvironmentDetection()


#arduino_serial
while True:
    #data = str(GPSser.readline())
    gps_data = GPSser.readline().decode('utf-8')
    if gps_data != '':
        gps_ret, robot_location = GPS.get_data(gps_data)
    data_raw = MotorSer.readline().decode('utf-8') #辨識迴圈中依然要接收Arduino按鈕
    data_raw = data_raw.replace("\n", "").replace("\r", "")
    if data_raw != '':
        print("Motor Input:",data_raw)
    if data_raw == 'R' and gps_ret == True:
        print("Road detection mode")
        #Google map目的地和終點的路徑規劃
        print(robot_location)
        way_point = GPS.directions(robot_location)
        #model setting
        model_inference = YOLACTEdgeInference(
            model_info.weights, model_info.config, model_info.dataset, model_info.calib_images, model_info.config_ovr)
        
        #camera setting
        init_params = sl.InitParameters()
        init_params.camera_fps = 60 # Set fps at 60
        init_params.camera_resolution = sl.RESOLUTION.HD720 # Set image size to 1280*720
        init_params.depth_minimum_distance = 0.30
        cam = sl.Camera()
        if not cam.is_opened():
            print("Opening ZED Camera...")
        status = cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()
        #recording setting
        output_path = sys.argv[0]
        recordingParameters = sl.RecordingParameters()
        recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
        recordingParameters.video_filename = "./results/HD720_TEST.svo"
        err = cam.enable_recording(recordingParameters)
        runtime = sl.RuntimeParameters()
        Left_image = sl.Mat()
        Depth_view = sl.Mat()
        err = cam.grab(runtime)
        ts_handler = TimestampHandler()
        sensors_data = sl.SensorsData()

        end1 = time.time()
        time_state = 0
        realFPS = []
        location_history = []
        while err == sl.ERROR_CODE.SUCCESS:
            """SS = time.time()
            if time_state <2:
                if time_state==0:
                    start10 = time.time()
                time_state = time_state+1
            if time_state == 2:
                end10 = time.time()
                realFPS = 1/(end10-start10)
                time_state = 0"""
            start = time.time()
            data_raw = MotorSer.readline().decode('utf-8') #辨識迴圈中依然要接收Arduino按鈕
            data_raw = data_raw.replace("\n", "").replace("\r", "")
            if data_raw != '':
                print("Motor Input:",data_raw)
                Motor = data_raw
            #接收Arduino物件搜尋的指令，並設定清空條件
            cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 100)
            err = cam.grab(runtime)
            if cam.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
                if ts_handler.is_new(sensors_data.get_imu_data()):
                    quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
                    fix_quaternion = quaternion[0]/0.009,quaternion[1]/0.009,quaternion[2]/0.009
                    angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
                    magnetometer_data = sensors_data.get_magnetometer_data()
                    magnetic_field = magnetometer_data.get_magnetic_field_calibrated()
                    magnetic = magnetometer_data.magnetic_heading
                    
                    if quaternion[0] > 0.1:
                        magnetic = magnetic + (quaternion[0]*100)
                        #print("angular too high")
                    elif quaternion[0] < -0.1:
                        magnetic = magnetic - (quaternion[0]*200)
                        #print("angular too low")
                        
            #ZED感測器
            gps_data = GPSser.readline().decode('utf-8')
            gps_ret, robot_location = GPS.get_data(gps_data)
            navigation_state, distance, angle = GPS.distance_to_point(robot_location)
            #如果在物件搜尋狀態，且上一幀已辨識到目標物件的位置，則現在把導航點覆蓋成物件所在的方向
            all_area_angle = scoring.area_angle(magnetic)
            gps_score,gps_rank,fix_angle_state,delta_angle = scoring.gps_scoreing(angle)      
                #計算前方7個(面向方位角)方位與目標方位角的修正角度，計分公式為:1-(修正角度/180)，即與目標角度相差的角度愈小，該方向愈高分
            #計算GPS定位    最快修正的轉向 fix_angle_state的1為順時鐘轉向 2為逆時鐘轉向
            cam.retrieve_image(Left_image, sl.VIEW.LEFT)
            cam.retrieve_image(Depth_view, sl.VIEW.DEPTH)
            #start = time.time()
            data_img = np.zeros((210,1280,3), dtype='uint8')
            Left_RGB = cv2.cvtColor(Left_image.get_data(), cv2.COLOR_RGBA2RGB)
            Cut_Left_RGB = Left_RGB[306:720,0:1280]
            O_Depth_GRAY = cv2.cvtColor(Depth_view.get_data(), cv2.COLOR_RGBA2GRAY)
            Depth_GRAY = O_Depth_GRAY[0:scoring.depth_h,0:1280]
            
            p = model_inference.predict(Cut_Left_RGB, True)
            if p != None:
                p_state = True
            else:
                p_state = False
            end1 = time.time()

            depth_area_score,depth_area_rank = scoring.depth_scoring(Depth_GRAY)
            
            if navigation_state == False:
                
                if p_state == True:
                    far_mask_precent,mid_mask_precent,close_mask_precent = scoring.mask_precent(p["class"],p["mask"])
                    mask_area_score, mask_area_rank = scoring.mask_scoring()
                    average_score, average_rank = scoring.final_scoring(p_state)
                    final_decision = scoring.final_decision()
                    
                    farline_class = robot_environment_decision.farline_detect(far_mask_precent)
                    midline_class = robot_environment_decision.midline_detect(mid_mask_precent)
                    closeline_class = robot_environment_decision.closeline_detect(close_mask_precent)
                
                    
                    arduino_msg = f'3_{final_decision}_0\n'
                    arduino_msg = robot_decision.vision_decision(arduino_msg,depth_area_score,mask_area_rank,mask_area_score)
                    arduino_msg = robot_environment_decision.detecting_environmental(arduino_msg) #探測線結果
                    arduino_msg = robot_decision.sensor_decision(fix_quaternion,delta_angle,fix_angle_state)
                    
                    end2 = time.time()
                    FPS1 = 1/(end1-start)
                    FPS2 = 1/(end2-start)
                    
                    p_img,Depth_GRAY = scoring.p_img_draw(p_state,p["img"],Depth_GRAY)
                    data_img = GPS.data_img_draw(data_img,delta_angle,magnetic)
                    data_img = robot_environment_decision.data_img_draw(data_img,far_mask_precent,mid_mask_precent,close_mask_precent)
                    cv2.putText(data_img, str("Predict FPS: {:.2f}".format(FPS1)), (20 , 20), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(data_img, str("All calculation FPS: {:.2f}".format(FPS2)), (20 , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(data_img, str("Quaternion: {0:.1f} | {1:.1f}".format(fix_quaternion[0],fix_quaternion[2])), (20 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    cv2.putText(data_img, str("To Arduino: {}".format(arduino_msg)), (350 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    #cv2.putText(data_img, str("Arduino case: {}".format(Motor)), (360 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    image = np.vstack((data_img,p["img"]))
                    cv2.imshow("Depth", Depth_GRAY)
                    cv2.imshow("image",image)
                    
                    MotorSer.write(arduino_msg.encode('ascii'))
                    
                else: #畫面沒有辨識結果的判斷邏輯
                    average_score, average_rank = scoring.final_scoring(p_state)
                    final_decision = scoring.final_decision()
                    
                    arduino_msg = f'3_{final_decision}_0\n'
                    arduino_msg = robot_decision.depth_detect(depth_area_score)
                    arduino_msg = robot_decision.sensor_decision(fix_quaternion,delta_angle,fix_angle_state)
                    cv2.putText(data_img, str("Quaternion: {0:.1f} | {1:.1f}".format(fix_quaternion[0],fix_quaternion[2])), (20 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    cv2.putText(data_img, str("To Arduino: {}".format(arduino_msg)), (350 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    
                    p_img,Depth_GRAY = scoring.p_img_draw(p_state,Cut_Left_RGB,Depth_GRAY)
                    data_img = GPS.data_img_draw(data_img,delta_angle,magnetic)
                    image = np.vstack((data_img,Cut_Left_RGB))
                    cv2.imshow("Depth", Depth_GRAY)
                    cv2.imshow("image",image)
                    
                    MotorSer.write(arduino_msg.encode('ascii'))
            else:
                arduino_msg=f'3_{200}_0\n'
                print('Finish and stop !')#case 20是執行停下和播放到達目的地的聲音
                    
                MotorSer.write(arduino_msg.encode('ascii'))
                #print(arduino_msg)
                #old_arduino_msg = arduino_msg
                #MotorSer.flush()
                #=================
            
            if cv2.waitKey(1) and data_raw == 'Q':#if data_raw == 'Q':
                data_raw = None
                break
            
        cv2.destroyAllWindows()
        cam.disable_recording()
        cam.close()
        print("\nFINISH")
        time.sleep(1)
 
    if data_raw == 'O':
    #     print("object detection mode")
    #     weights = "./weights/yolact_edge_resnet50_54_800000.pth"
    #     config = 'yolact_resnet50_config'
    #     dataset = 'dataset_base'
    #     calib_images = "./data/coco/calib_images"
    #     images_num = os.listdir(calib_images)
    #     images_path = len(images_num)
    #     config_ovr = {
    #         'use_fast_nms': True,  # Does not work with regular nms
    #         'mask_proto_debug': False
    #     }
    #     model_inference = YOLACTEdgeInference(
    #         weights, config, dataset, calib_images, config_ovr)
    #     #model setting============================================

    #     init_params = sl.InitParameters()
    #     init_params.camera_fps = 100 # Set fps at 100
    #     init_params.camera_resolution = sl.RESOLUTION.VGA # Set image size to 672*376
    #     cam = sl.Camera()
    #     if not cam.is_opened():
    #         print("Opening ZED Camera...")
    #     status = cam.open(init_params)
    #     if status != sl.ERROR_CODE.SUCCESS:
    #         print(repr(status))
    #         exit()

    #     runtime = sl.RuntimeParameters()
    #     Left_image = sl.Mat()
    #     Depth_view = sl.Mat()
    #     err = cam.grab(runtime)
    #     #camera setting===========================================
    #     area_cut = 14
    #     area_w = int(672/area_cut)
    #     area_h = int(376)
    #     area_start_y = 0
    #     max_distance = 230
    #     while err == sl.ERROR_CODE.SUCCESS:
    #         data_raw = MotorSer.readline().decode('utf-8') #辨識迴圈中依然要接收Arduino按鈕
    #         data_raw = data_raw.replace("\n", "").replace("\r", "")
    #         err = cam.grab(runtime)
    #         cam.retrieve_image(Left_image, sl.VIEW.LEFT)
    #         cam.retrieve_image(Depth_view, sl.VIEW.DEPTH)
    #         Left_RGB = cv2.cvtColor(Left_image.get_data(), cv2.COLOR_RGBA2RGB)
    #         Depth_GRAY = cv2.cvtColor(Depth_view.get_data(), cv2.COLOR_RGBA2GRAY)
    #         cv2.imshow("Depth", Depth_GRAY)
    #         p = model_inference.predict(Left_RGB, True)
    #         if p != None:
    #             cv2.imshow("ZED", p["img"])
    #         else:
    #             cv2.imshow("ZED", Left_RGB)
                
    #         if cv2.waitKey(1) and data_raw == 'Q':
    #             break
            
    #     cv2.destroyAllWindows()

    #     cam.close()
    #     print("\nFINISH")
    #     time.sleep(1)
        
    # if data_raw == 'M':
    #     print("object searching mode")
    #     weights = "./weights/yolact_edge_resnet50_54_800000.pth"
    #     config = 'yyolact_resnet50_config'
    #     dataset = 'dataset_base'
    #     calib_images = "./data/coco/calib_images"
    #     images_num = os.listdir(calib_images)
    #     images_path = len(images_num)
    #     config_ovr = {
    #         'use_fast_nms': True,  # Does not work with regular nms
    #         'mask_proto_debug': False
    #     }
    #     args_ovr = {'score_threshold': 0.6}
    #     model_inference = YOLACTEdgeInference(
    #         weights, config, dataset, calib_images, config_ovr, args_ovr)
    #     #model setting============================================
    #     cam = sl.Camera()
    #     init_params = sl.InitParameters()
    #     init_params.camera_fps = 30 # Set fps at 60
    #     init_params.camera_resolution = sl.RESOLUTION.HD720 # Set image size to 1280*720
    #     init_params.depth_minimum_distance = 0.30
    #     init_params.coordinate_units = sl.UNIT.MILLIMETER
    #     init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    #     #若要取得某一點的32位浮點數的準確距離值
    #     if not cam.is_opened():
    #         print("Opening ZED Camera...")
    #     status = cam.open(init_params)
    #     if status != sl.ERROR_CODE.SUCCESS:
    #         print(repr(status))
    #         exit()
    #     output_path = sys.argv[0]
    #     '''
    #     recordingParameters = sl.RecordingParameters()
    #     recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    #     recordingParameters.video_filename = "./results/HD720_TEST_OSM.svo"
    #     err = cam.enable_recording(recordingParameters)
    #     '''
    #     #recording code==========================================
    #     runtime = sl.RuntimeParameters()
    #     Left_image = sl.Mat()
    #     Depth_view = sl.Mat()
    #     Measure_Depth_view = sl.Mat()
    #     err = cam.grab(runtime)
    #     ts_handler = TimestampHandler()
    #     sensors_data = sl.SensorsData()
    #     #camera setting===========================================
    #     search_object = "None"
    #     area_cut = 7
    #     area_w = int(1280/area_cut)
    #     mask_h = int(720)
    #     depth_h = int(550)
    #     area_start_y = 0
    #     max_distance = 240
    #     GPS_score = [0,0,0,0,0,0,0]
    #     GPS_rank = [0,0,0,0,0,0,0]
    #     all_area_angle = [0,0,0,0,0,0,0]
    #     #area setting=============================================
    #     ID = 3
    #     pose_state = 0
    #     old_pose_state=[]
    #     fix_angle_state = 0
    #     turn_back_state = 0
    #     total_blocked_time = 0
    #     lose_pre_t = 0
    #     lose_pre_state = 0
    #     qua_state = 0
    #     TOF_state=0
    #     searching_state=0
    #     Closest_Obj_area=0
    #     turn_time=0
    #     old_object_relat=0
    #     angle = 0
    #     track_state = 0
    #     relat_obj_num = -1
    #     relat_obj_rank = -1
    #     old_relat_obj_num = 30
    #     old_relat_obj_rank = 30
    #     miss_relat_time = 0
    #     found_obj_num = []
    #     found_obj_rank = []
    #     found_obj = []
    #     data_raw_fix = []
    #     target_obj_num = -1
    #     start_angle = []
    #     scaning_state = 0
    #     turn_state = 0
    #     arduino_msg = str("None\n")
    #     depth_value = []
    #     center_point = []
    #     #arduino setting===========================================
    #     COCO_CLASSES2NUM = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'}
    #     #person_list = [25, 67, 56, 79, 43, 63, 61, 45, 57, 74, 62, 40, 42, 78, 65, 60, 59, 41, 44, 66, 76, 72, 64, 58, 69, 71, 75, 68, 70]
    #     chair_list = [60, 25, 0, 63, 43, 58, 45, 40, 69, 67, 65, 57, 42, 72, 68, 66, 41, 62, 74, 76, 79, 75, 44, 59, 64, 78, 71, 61, 70]
    #     umbrella_list = [56, 0, 60, 43, 40, 58, 42, 76, 44, 79, 67, 45, 41, 63, 68, 74, 78, 69, 66, 70, 61, 59, 71, 65, 72, 64, 57, 62, 75]
    #     tv_list = [63, 65, 67,  57, 75, 66, 76, 61, 58, 56, 60, 78, 68, 79, 70, 74, 40, 64, 69, 59, 44, 42, 25, 71, 45, 72, 43, 41]
    #     #search setting==========================================
        
    #     while err == sl.ERROR_CODE.SUCCESS:
    #         target_box = []
    #         relat_box = []
    #         mask_point = []
    #         data_raw = MotorSer.readline().decode('utf-8') #辨識迴圈中依然要接收Arduino按鈕
    #         data_raw = data_raw.replace("\n", "").replace("\r", "")
    #         if data_raw != '':
    #             print("motordata:",data_raw)
    #         if data_raw == 'C' and search_object == "None":
    #             search_object = "Chair"
    #             target_obj_num = 56
    #             search_list_name = "Chair list"
    #             search_list = list(chair_list)
    #             print("searching Chair")
    #         if data_raw == 'B' and search_object == "None":
    #             search_object = "Ubrella"
    #             target_obj_num = 25
    #             search_list_name = "umbrella_list"
    #             search_list = list(umbrella_list)
    #             print("searching Umbrella")
    #         if data_raw == 'P' and search_object == "None":
    #             search_object = "TV"
    #             target_obj_num = 62
    #             search_list_name = "tv_list"
    #             search_list = list(tv_list)
    #             print("searching TV")
    #         if data_raw == 'end' :
    #             search_object = "None"
    #             found_obj = []
    #             track_state = 0
    #             turn_time = 0
    #             scaning_state = 0
    #             searching_state = 0
    #             target_obj_num = -1
    #             relat_obj_num = -1
    #             relat_obj_rank = -1
    #             old_relat_obj_num = 30
    #             old_relat_obj_rank = 30
    #         #接收Arduino物件搜尋的指令，並設定清空條件
    #         err = cam.grab(runtime)
    #         if cam.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
    #             if ts_handler.is_new(sensors_data.get_imu_data()):
    #                 quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
    #                 fix_quaternion = quaternion[0]/0.009,quaternion[1]/0.009,quaternion[2]/0.009
    #                 angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
    #                 magnetometer_data = sensors_data.get_magnetometer_data()
    #                 magnetic_field = magnetometer_data.get_magnetic_field_calibrated()
    #                 magnetic = magnetometer_data.magnetic_heading
    #                 if searching_state == 0 and track_state == 0:
    #                     angle = magnetic
    #                 if quaternion[0] > 0.1:
    #                     magnetic = magnetic + (quaternion[0]*100)
    #                     #print("angular too high")
    #                 elif quaternion[0] < -0.1:
    #                     magnetic = magnetic - (quaternion[0]*200)
    #                     #print("angular too low")
                        
    #         for GPS_num in range(area_cut):
    #             if GPS_num < 3:
    #                 GPS_score[GPS_num] = magnetic+17*-(3-GPS_num)
    #             elif GPS_num >3 :
    #                 GPS_score[GPS_num] = magnetic+17*-(3-GPS_num)
    #             elif GPS_num ==3 :
    #                 GPS_score[GPS_num] = magnetic
    #         for GPS_num in range(area_cut):
    #             if abs(GPS_score[GPS_num])>180 and GPS_score[GPS_num] >-129:
    #                 GPS_score[GPS_num] = -180-(180-GPS_score[GPS_num])
    #             elif abs(GPS_score[GPS_num])>180 and GPS_score[GPS_num] <-129:
    #                 GPS_score[GPS_num] = 180-(abs(180+GPS_score[GPS_num]))
            
    #         all_area_angle = list(np.round(GPS_score))
            
    #         #如果在物件搜尋狀態，且上一幀已辨識到目標物件的位置，則現在把導航點覆蓋成物件所在的方向
    #         for GPS_num in range(area_cut):
    #             if angle >=0 and GPS_score[GPS_num] >=0:
    #                 fix_angle=abs(angle-GPS_score[GPS_num])
    #                 GPS_score[GPS_num]=1-fix_angle/180
    #                 if GPS_num == 3:
    #                     delta_angle = fix_angle
    #                     if all_area_angle[GPS_num] > angle :
    #                         fix_angle_state=1
    #                     else:
    #                         fix_angle_state=2
    #             elif angle >=0 and GPS_score[GPS_num] <=0:
    #                 if GPS_score[GPS_num] > -(180-angle) :
    #                     fix_angle=abs(angle-GPS_score[GPS_num])
    #                     GPS_score[GPS_num]=1-fix_angle/180
    #                     if GPS_num == 3:
    #                         delta_angle = fix_angle
    #                         fix_angle_state=2
    #                 else:
    #                     fix_angle=abs(180+GPS_score[GPS_num]+180-angle)
    #                     GPS_score[GPS_num]=1-fix_angle/180
    #                     if GPS_num == 3:
    #                         delta_angle = fix_angle
    #                         fix_angle_state=1
    #             elif angle <=0 and GPS_score[GPS_num] <=0:
    #                 fix_angle=abs(angle-GPS_score[GPS_num])
    #                 GPS_score[GPS_num]=1-fix_angle/180
    #                 if GPS_num == 3:
    #                     delta_angle = fix_angle
    #                     if all_area_angle[GPS_num] > angle :
    #                         fix_angle_state=1
    #                     else:
    #                         fix_angle_state=2
    #             elif angle <=0 and GPS_score[GPS_num] >=0:
    #                 if GPS_score[GPS_num] > (180+angle) :
    #                     fix_angle=abs(180+angle+(180-GPS_score[GPS_num]))
    #                     GPS_score[GPS_num]=1-fix_angle/180
    #                     if GPS_num == 3:
    #                         if  all_area_angle[GPS_num] > (180+angle) :
    #                             delta_angle = fix_angle
    #                             fix_angle_state=1
    #                         else:
    #                             delta_angle = fix_angle
    #                             fix_angle_state=2
                            
    #                 else:
    #                     fix_angle=abs(-angle+GPS_score[GPS_num])
    #                     GPS_score[GPS_num]=1-fix_angle/180
    #         GPS_score = np.array(GPS_score)
    #             #計算前方7個(面向方位角)方位與目標方位角的修正角度，計分公式為:1-(修正角度/180)，即與目標角度相差的角度愈小，該方向愈高分
    #         #計算GPS定位    最快修正的轉向 fix_angle_state的1為順時鐘轉向 2為逆時鐘轉向
    #         cam.retrieve_image(Left_image, sl.VIEW.LEFT)
    #         cam.retrieve_image(Depth_view, sl.VIEW.DEPTH)
    #         cam.retrieve_measure(Measure_Depth_view, sl.MEASURE.DEPTH)
    #         start = time.time()
    #         data_img = np.zeros((210,1280,3), dtype='uint8')
    #         Left_RGB = cv2.cvtColor(Left_image.get_data(), cv2.COLOR_RGBA2RGB)
    #         O_Depth_GRAY = cv2.cvtColor(Depth_view.get_data(), cv2.COLOR_RGBA2GRAY)
    #         Depth_GRAY = O_Depth_GRAY[0:depth_h,0:1280]
    #         p = model_inference.predict(Left_RGB, True)
    #         end1 = time.time()
    #         Mask_area_score = np.zeros(area_cut)
    #         Mask_area_rank = np.zeros(area_cut)
    #         Depth_area_score = np.zeros(area_cut)
    #         Depth_area_rank = np.zeros(area_cut)
    #         average_score = np.zeros(area_cut)
    #         average_rank = np.zeros(area_cut)
    #         for area_num in range(area_cut):#每一個Mask和Depth_map分成7等分
    #             area_start_x = area_w*(area_num)+area_num
    #             area_end_x = area_w*(area_num+1)+area_num
    #             Depth_area = Depth_GRAY[area_start_y:depth_h,area_start_x:area_end_x]
    #             if np.max(Depth_area) >max_distance: #當depth map像素距離到閥值(過近) 分數為-10
    #                 Depth_area_score[area_num] = -1
    #             else :
    #                 Depth_area_score[area_num] = Depth_area_score[area_num]+(1-(np.max(Depth_area)/255)) #前方愈空曠愈高分
    #         if p != None:
    #             for mask_num in range(len(p["class"])):
    #                 area_start_x = 0
    #                 area_end_x = 0
    #                 if len(p["class"]) == 1:
    #                     tensor_mask = p["mask"]
    #                     tensor_mask = np.uint8((tensor_mask).cpu())
    #                     class_num = int((p["class"]))
    #                     if search_object == "Chair" or search_object == "TV" or search_object == "Umbrella":
    #                         if class_num == target_obj_num:
    #                             target_box.append(p["boxes"][0])
    #                             mask_point = np.where(tensor_mask == 1)
    #                             searching_state = 1
    #                             track_state = 0
    #                         if class_num in search_list and searching_state != 1:
    #                             relat_obj_num = class_num
    #                             relat_obj_rank = search_list.index(class_num)
    #                             if relat_obj_rank <= old_relat_obj_rank:
    #                                 old_relat_obj_num = relat_obj_num
    #                                 old_relat_obj_rank = relat_obj_rank
    #                                 target_box.append(p["boxes"][0])
    #                                 mask_point = np.where(tensor_mask == 1)
    #                                 track_state = 1
    #                     #如果處於掃瞄狀態，掃到存在於search_list的物件會進行比較
    #                     #如果掃到的物件是目前看到最高分的物件並位於正前方，便會紀錄物件的方位角
    #                 else:
    #                     tensor_mask = p["mask"][mask_num]
    #                     tensor_mask = np.uint8((tensor_mask).cpu())
    #                     class_num = int((p["class"][mask_num]))
    #                     if search_object == "Chair" or search_object == "TV" or search_object == "Umbrella":
    #                         if class_num == target_obj_num:
    #                             target_box.append(p["boxes"][mask_num])
    #                             mask_point.append(np.where(tensor_mask == 1))
    #                             searching_state = 1
    #                         if class_num in search_list and searching_state != 1:
    #                             relat_obj_num = class_num
    #                             relat_obj_rank = search_list.index(class_num)
    #                             if relat_obj_rank <= old_relat_obj_rank:
    #                                 old_relat_obj_num = relat_obj_num
    #                                 old_relat_obj_rank = relat_obj_rank
    #                                 target_box.append(p["boxes"][mask_num])
    #                                 mask_point.append(np.where(tensor_mask == 1))
    #                                 track_state = 1

    #                 for area_num in range(area_cut):#每一個Mask分成7等分
    #                     area_start_x = area_w*(area_num)+area_num
    #                     area_end_x = area_w*(area_num+1)+area_num
    #                     mask_area = tensor_mask[area_start_y:mask_h,area_start_x:area_end_x]
                        
    #                     if class_num == target_obj_num or class_num == old_relat_obj_num: #除了target or relat  加分 其他class的NO加分
    #                         Mask_area_score[area_num] = Mask_area_score[area_num]+np.average(mask_area)
    #             if search_object != "None":
    #                 center_point = []
    #                 random10_mask_point = []
    #                 center_distance = []
    #                 point_distance = []
    #                 target_distance = []
    #                 min_obj_distance = []
    #                 if target_box != [] :
    #                     if len(target_box) ==1 :
    #                         x=int((target_box[0][2]-target_box[0][0])/2+target_box[0][0])
    #                         y=int((target_box[0][3]-target_box[0][1])/2+target_box[0][1])
    #                         y1=int(target_box[0][1]+15)
    #                         y2=int(target_box[0][3]-15)
    #                         center_point.append([x,y])
    #                         center_point.append([x,y1])
    #                         center_point.append([x,y2])
    #                         target_area = np.max((O_Depth_GRAY[target_box[0][1]:target_box[0][3],target_box[0][0]:target_box[0][2]]))
    #                         target_distance.append(target_area)
    #                         for box_num in range(len(mask_point)):
    #                             error0 ,depth_value = Measure_Depth_view.get_value(center_point[0][0], center_point[0][1]) 
    #                             center_distance.append(depth_value)
    #                     else:
    #                         for box_num in range(len(target_box)):
    #                             x=int((target_box[box_num][2]-target_box[box_num][0])/2+target_box[box_num][0])
    #                             y=int((target_box[box_num][3]-target_box[box_num][1])/2+target_box[box_num][1])
    #                             y1=int(target_box[box_num][1]+15)
    #                             y2=int(target_box[box_num][3]-15)
    #                             center_point.append([x,y])
    #                             center_point.append([x,y1])
    #                             center_point.append([x,y2])
    #                             target_area = np.max((O_Depth_GRAY[target_box[box_num][1]:target_box[box_num][3],target_box[box_num][0]:target_box[box_num][2]]))
    #                             target_distance.append(target_area)
    #                         for box_num in range(len(center_point)):
    #                             error0 ,depth_value = Measure_Depth_view.get_value(center_point[box_num][0], center_point[box_num][1]) 
    #                             center_distance.append(depth_value)
    #                     center_distance = list(np.round(center_distance)/10) 
    #                     Closest_Obj_area = int(center_point[np.argmin(center_distance)][0]/182)#計算最接近的一個目標物件位於7個area中的哪一塊
    #                     angle = int(all_area_angle[Closest_Obj_area]) #calculate box center distance(able to try calculate center point up,down,left,right 10 pixel, and get the min distance)
    #                     save_angle = angle
                        
    #             GPS_rank = np.argsort(GPS_score)
    #             GPS_rank = np.lexsort((GPS_score,GPS_rank))
                
    #             Depth_area_score = np.array(Depth_area_score)
    #             Depth_area_rank = np.argsort(Depth_area_score)
    #             Depth_area_rank = np.lexsort((Depth_area_score,Depth_area_rank))
    #             Depth_area_rank = np.where(Depth_area_score>0,Depth_area_rank,-5)
                
    #             average_score = GPS_rank+Depth_area_rank
    #             average_rank = np.argsort(average_score)
    #             average_rank = np.lexsort((average_score,average_rank))
            
    #             Highest_score_area = np.argmax(average_rank)
    #             Input = Highest_score_area*20
                
    #             if target_box != [] :
    #                 arduino_msg=str(ID)+"_"+str(int(Closest_Obj_area)*20)+"_"+str(int(Input))+'\n'
    #                 if  np.nanmin(center_distance) <100 and Closest_Obj_area !=3 :#
    #                     arduino_msg=f'{ID}_140_0\n'
    #                     print("target object too close")
    #                     #更足夠接近物件且物件位於正前方，就停下並發送找到的訊號
    #                     if Closest_Obj_area <3 :
    #                         arduino_msg=f'{ID}_150_0\n'
    #                         print("target object in your left")
    #                     if Closest_Obj_area >3 :
    #                         arduino_msg=f'{ID}_160_0\n'
    #                         print("target object in your right")
    #                 #if average_score[Closest_Obj_area] >0:
    #                 #    arduino_msg=str(ID)+"_"+str(int(Closest_Obj_area))+"_"+str(int(0))+'\n'
    #                 #    print("following object")
                    
    #                 elif np.nanmin(center_distance) <=100 and Closest_Obj_area ==3 and search_object != "None":#
    #                     if searching_state == 1 and track_state == 0:
    #                         arduino_msg=f'{ID}_130_0\n'
    #                         TOF_state = TOF_state+1
    #                         #更足夠接近物件且物件位於正前方，就停下確認是否連續5幀的距離都到達閥值
    #                         if TOF_state >5:
    #                             arduino_msg=f'{ID}_200_0\n'
    #                             print("found target object")
    #                             qua_state = qua_state +1
    #                             TOF_state=0
    #                             searching_state = 0
    #                             if qua_state>5:
    #                                 data_raw = 'Q'
    #                                 print("Quiting searching mode")
    #                     elif searching_state == 0 and track_state == 1:
    #                         arduino_msg=f'{ID}_130_0\n'
    #                         TOF_state = TOF_state+1
    #                     #更足夠接近物件且物件位於正前方，就停下確認是否連續5幀的距離都到達閥值
    #                         if TOF_state >5:
    #                             #arduino_msg=f'{ID}_130_0\n'
    #                             print("found relat object")
    #                             found_obj.append(COCO_CLASSES2NUM[search_list[old_relat_obj_rank]])
    #                             found_obj_num.insert(0,old_relat_obj_num)
    #                             found_obj_rank.insert(0,old_relat_obj_rank)
    #                             search_list.remove(search_list[old_relat_obj_rank])
    #                             qua_state = 0
    #                             TOF_state = 0
    #                             scaning_state = 0
    #                             track_state = 0
    #                         #停下並發送找到的訊號，並清空物件搜尋的變數
    #                 elif  np.min(center_distance) == 'nan':#
    #                     arduino_msg=f'{ID}_130_0\n'
    #                     print("min distance is nan")
    #                 #如果距離數值回傳nan 則停下
                    
    #                 #如有多個目標物件，比較何者最接近
    #                 #並計算目標物件辨識框的中心點位於7個方向中的哪一格
                    
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state == 0: #深度圖中7個area全部都到閥值 回頭狀態為0 執行停下等待2秒(等移動障礙物自行離開)
    #                 blocked_time = time.time()
    #                 arduino_msg=f'{ID}_130_0\n'
    #                 print('Depth noway to go')
    #                 turn_back_state = turn_back_state+1
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state==1:#深度圖中7個area全部都到閥值 回頭狀態為1 
    #                 new_blocked_time = time.time()
    #                 total_blocked_time = new_blocked_time-blocked_time
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state==1 and total_blocked_time > 2:#深度圖中7個area全部都到閥值  阻擋時間大於2秒 回頭狀態+1
    #                 turn_back_state = turn_back_state+1
    #                 blocked_time = time.time()
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state == 2:#深度圖中7個area全部都到閥值 回頭狀態為2 
    #                 new_blocked_time = time.time()
    #                 total_blocked_time = new_blocked_time-blocked_time

    #             #畫面判斷訊息
    #             if fix_quaternion[0] > 10 and pose_state == 0:
    #                 print('Going up')
    #                 arduino_msg = f'{ID}_180_0\n'
    #                 qua_state = qua_state+1
    #                 if qua_state > 3:
    #                     pose_state = 1
    #                     qua_state = 0
    #             elif fix_quaternion[0] < 5 and pose_state == 1:
    #                 pose_state = 0
    #             elif fix_quaternion[0] < -10 and pose_state == 0:
    #                 print('Going down')
    #                 arduino_msg = f'{ID}_170_0\n'
    #                 if qua_state > 3:
    #                     pose_state = -1
    #                     qua_state = 0
    #             elif fix_quaternion[0] > -5 and pose_state == -1:
    #                 pose_state = 0
    #             #陀螺儀感測動作
                
    #             if scaning_state == 0 and search_object != "None":
    #                 arduino_msg = f'{ID}_150_0\n'
    #                 print("scaning round")
    #                 turn_time = turn_time+1
    #                 if turn_time == 1:
    #                     start_angle=int(magnetic)
    #                 elif turn_time > 100 and found_obj_rank == []: 
    #                     if abs(int(magnetic))-abs(start_angle) < 10 and abs(int(magnetic))-abs(start_angle) > -10 :
    #                         print("{0} | {1}".format(magnetic,start_angle))
    #                         scaning_state = 1
    #                         turn_time = 0
    #                         if searching_state == 0 and track_state == 0:
    #                             data_raw = 'Q'
    #                             print("unable to find any relat object!")
    #                 elif turn_time > 100 and found_obj_rank != []:
    #                     if abs(int(magnetic))-abs(start_angle) < 90 and abs(int(magnetic))-abs(start_angle) > -90 :
    #                         print("{0} | {1}".format(magnetic,start_angle))
    #                         scaning_state = 1
    #                         turn_time = 0
    #                         if searching_state == 0 and track_state == 0:
    #                             data_raw = 'Q'
    #                             print("unable to find any relat object!")
    #             #如果還沒掃瞄過 且有輸入搜尋物件則進行原地旋轉      
    #             if  data_raw == "end" or search_object == "None":
    #                 print("waiting target input")
    #                 arduino_msg = f'None\n'
                    
    #             if scaning_state == 1 and delta_angle>45:
    #                 if searching_state == 1 or track_state == 1:
    #                     turn_state = 1
    #                 if turn_state ==1 and delta_angle>5:
    #                     if fix_angle_state == 1:
    #                         arduino_msg=f'{ID}_150_0\n'
    #                         print('turning left')
    #                     if fix_angle_state == 2:
    #                         arduino_msg=f'{ID}_160_0\n'
    #                         print('turning right')
    #                 elif turn_state ==1 and delta_angle<5:
    #                      turn_state = 0
    #             #如果已經掃瞄過 且與物件的角度差超過18度則原地旋轉至物件紀錄的角度
                
    #             MotorSer.write(arduino_msg.encode('ascii'))
    #             print(arduino_msg)
    #             end2 = time.time()
    #             FPS1 = 1/(end1-start)
    #             FPS2 = 1/(end2-start)
    #             if Closest_Obj_area != 'nan':
    #                 cv2.circle(p["img"],((area_w*(Closest_Obj_area+1)-91),170),5,(0,255,0),3)
    #             for score_num in range(area_cut):
    #                 cv2.line(p["img"], (area_w*(score_num+1)+score_num,0), (area_w*(score_num+1)+score_num,720), (0, 255, 255), 1)
    #                 cv2.line(Depth_GRAY, (area_w*(score_num+1)+score_num,0), (area_w*(score_num+1)+score_num,720), (255), 1)
    #                 cv2.putText(p["img"], str("{:.2f}".format(GPS_score[score_num])), ((60+area_w*(score_num)) , 20), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(p["img"], str("{}".format(GPS_rank[score_num])), ((60+area_w*(score_num)) , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(p["img"], str("{:.2f}".format(Depth_area_score[score_num])), ((60+area_w*(score_num)) , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Predict FPS: {:.2f}".format(FPS1)), (20 , 20), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("All calculation FPS: {:.2f}".format(FPS2)), (20 , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Searching object: {0}({1})".format(search_object,target_obj_num )), (20 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Target object angle: {:.2f}".format(angle)), (20 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Face angle: {0:.2f}".format(magnetic)), (20 , 100), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("delta angle: {0:.1f}".format(delta_angle)), (20 , 120), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Quaternion: {0:.1f} | {1:.1f}".format(fix_quaternion[0],fix_quaternion[2])), (20 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("class: {}".format(p["class"])), (350 , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if relat_box != []:
    #                 cv2.putText(data_img, str("Depth Value: {0} | TBOX {1} | RBOX {2}".format(depth_value,target_box,relat_box)), (350 , 20), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if old_relat_obj_num != -1:
    #                 cv2.putText(data_img, str("Relat Object rank: {0} | {1}({2})".format((old_relat_obj_rank),COCO_CLASSES2NUM[old_relat_obj_num],old_relat_obj_num)), (350 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             else:
    #                 cv2.putText(data_img, str("Relat Object rank: {0} | {1}".format(old_relat_obj_rank,"None")), (350 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if search_object != "None":
    #                 cv2.putText(data_img, str("Search list name: {0} {1}".format(search_list_name,search_list)), (350 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(data_img, str("Relat Object: {0} {1} {2} {3}".format(relat_obj_num ,relat_obj_rank,old_relat_obj_num,old_relat_obj_rank)), (350 , 160), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("7 face angle: {}".format(all_area_angle)), (350 , 100), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Object on screen: {}".format(Closest_Obj_area)), (350 , 120), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Searching | Track State: {0} {1}".format(searching_state ,track_state)), (350 , 140), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("fix_angle_state: {0} {1}".format(turn_state ,fix_angle_state)), (350 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("To Arduino: {}".format(arduino_msg)), (350 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if center_point != []:
    #                 cv2.putText(data_img, str("center point: {}".format(center_point)), (650 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(data_img, str("min distance:{0}cm({1})".format(np.nanmin(center_distance),center_distance)), (650 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if found_obj !=[]:
    #                 cv2.putText(data_img, str("found object: {}".format(found_obj)), (650 , 160), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             found_obj
                
    #             image = np.vstack((data_img,p["img"]))
    #             #cv2.imshow("O_Depth", O_Depth_GRAY)
    #             cv2.imshow("Depth", Depth_GRAY)
    #             cv2.imshow("image", image)
    #             if delta_angle<5:
    #                 if str(old_relat_obj_num) not in str(class_num) or str(target_obj_num) !=str(old_relat_obj_num):
    #                     miss_relat_time = miss_relat_time +1
    #                     print(miss_relat_time)
    #                     if miss_relat_time >5:
    #                         if found_obj_rank == []:
    #                             miss_relat_time = 0
    #                             track_state = 0
    #                             searching_state = 0
    #                             old_relat_obj_rank = 30
    #                             old_relat_obj_num = 30
    #                         else:
    #                             miss_relat_time = 0
    #                             searching_state = 0
    #                             old_relat_obj_rank = found_obj_rank[0]
    #                             old_relat_obj_num = 30
    #         else:
    #             Highest_score_area = np.argmax(GPS_score)
    #             Input = Highest_score_area*20
    #             if delta_angle<5:
    #                 miss_relat_time = miss_relat_time +1
    #                 if miss_relat_time >10:
    #                     if found_obj_rank == []:
    #                         miss_relat_time = 0
    #                         track_state = 0
    #                         searching_state = 0
    #                         old_relat_obj_rank = 30
    #                         old_relat_obj_num = 30
    #                     else:
    #                         miss_relat_time = 0
    #                         searching_state = 0
    #                         old_relat_obj_rank = found_obj_rank[0]
    #                         old_relat_obj_num = 30
                    
    #             if np.sum(Depth_area_score)==-20 and turn_back_state == 0: #深度圖中7個area全部都到閥值 回頭狀態為0 執行停下等待2秒(等移動障礙物自行離開)
    #                 blocked_time = time.time()
    #                 arduino_msg=f'{ID}_130_0\n'
    #                 print('Depth noway to go')
    #                 turn_back_state = turn_back_state+1
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state==1:#深度圖中7個area全部都到閥值 回頭狀態為1 
    #                 new_blocked_time = time.time()
    #                 total_blocked_time = new_blocked_time-blocked_time
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state==1 and total_blocked_time > 2:#深度圖中7個area全部都到閥值  阻擋時間大於2秒 回頭狀態+1
    #                 turn_back_state = turn_back_state+1
    #                 blocked_time = time.time()
    #             elif np.sum(Depth_area_score)==-20 and turn_back_state == 2:#深度圖中7個area全部都到閥值 回頭狀態為2 
    #                 new_blocked_time = time.time()
    #                 total_blocked_time = new_blocked_time-blocked_time

    #             #畫面判斷訊息
    #             if fix_quaternion[0] > 10 and pose_state == 0:
    #                 print('Going up')
    #                 arduino_msg = f'{ID}_180_0\n'
    #                 qua_state = qua_state+1
    #                 if qua_state > 3:
    #                     pose_state = 1
    #                     qua_state = 0
    #             elif fix_quaternion[0] < 10 and pose_state == 1:
    #                 pose_state = 0
    #             elif fix_quaternion[0] < -10 and pose_state == 0:
    #                 print('Going down')
    #                 arduino_msg = f'{ID}_170_0\n'
    #                 if qua_state > 3:
    #                     pose_state = -1
    #                     qua_state = 0
    #             elif fix_quaternion[0] > -10 and pose_state == -1:
    #                 pose_state = 0
    #             #陀螺儀感測動作
                
    #             if scaning_state == 0 and search_object != "None":
    #                 arduino_msg = f'{ID}_150_0\n'
    #                 print("scaning round")
    #                 turn_time = turn_time+1
    #                 if turn_time == 1:
    #                     start_angle=int(magnetic)
    #                 elif turn_time > 100 and found_obj_rank == []: 
    #                     if abs(int(magnetic))-abs(start_angle) < 10 and  abs(int(magnetic))-abs(start_angle) > -10 :
    #                         print("{0} | {1}".format(magnetic,start_angle))
    #                         scaning_state = 1
    #                         turn_time = 0
    #                         if searching_state == 0 and track_state == 0:
    #                             data_raw = 'Q'
    #                             print("unable to find any relat object!")
    #                 elif turn_time > 100 and found_obj_rank != []:
    #                     if abs(int(magnetic))-abs(start_angle) < 90 and abs(int(magnetic))-abs(start_angle) > -90 :
    #                         print("{0} | {1}".format(magnetic,start_angle))
    #                         scaning_state = 1
    #                         turn_time = 0
    #                         if searching_state == 0 and track_state == 0:
    #                             data_raw = 'Q'
    #                             print("unable to find any relat object!")
    #             #如果還沒掃瞄過 且有輸入搜尋物件則進行原地旋轉      
    #             if  data_raw == "end" or search_object == "None":
    #                 print("waiting target input")
    #                 arduino_msg = f'None\n'
                    
    #             if scaning_state == 1 and delta_angle>45:
    #                 if searching_state == 1 or track_state == 1:
    #                     turn_state = 1
    #                 if turn_state ==1 and delta_angle>5:
    #                     if fix_angle_state == 1:
    #                         arduino_msg=f'{ID}_150_0\n'
    #                         print('turning left')
    #                     if fix_angle_state == 2:
    #                         arduino_msg=f'{ID}_160_0\n'
    #                         print('turning right')
    #                 elif turn_state ==1 and delta_angle<5:
    #                      turn_state = 0
    #             #如果已經掃瞄過 且與物件的角度差超過18度則原地旋轉至物件紀錄的角度
                
    #             MotorSer.write(arduino_msg.encode('ascii'))
    #             print(arduino_msg)
    #             for score_num in range(area_cut):
    #                 cv2.line(Left_RGB, (area_w*(score_num+1)+score_num,0), (area_w*(score_num+1)+score_num,720), (0, 255, 255), 1)
    #                 cv2.line(Depth_GRAY, (area_w*(score_num+1)+score_num,0), (area_w*(score_num+1)+score_num,720), (255), 1)
    #                 cv2.putText(Left_RGB, str("{:.2f}".format(GPS_score[score_num])), ((60+area_w*(score_num)) , 20), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(Left_RGB, str("{}".format(GPS_rank[score_num])), ((60+area_w*(score_num)) , 40), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(Left_RGB, str("{:.2f}".format(Depth_area_score[score_num])), ((60+area_w*(score_num)) , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
    #                 cv2.putText(Left_RGB, str("{}".format(Depth_area_rank[score_num])), ((60+area_w*(score_num)) , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 0), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Searching object: {}".format(search_object)), (20 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Target object angle: {:.2f}".format(angle)), (20 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Face angle: {0:.2f}".format(magnetic)), (20 , 100), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("delta angle: {0:.1f}".format(delta_angle)), (20 , 120), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("Quaternion: {0:.1f} | {1:.1f}".format(fix_quaternion[0],fix_quaternion[2])), (20 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("{0:.1f}, {1:.1f}, {2:.1f}".format(angular_velocity[0], angular_velocity[1], angular_velocity[2])), (20 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    #             if relat_obj_num != -1:
    #                 cv2.putText(data_img, str("Relat Object: {0} | {1}({2})".format(old_relat_obj_rank,COCO_CLASSES2NUM[old_relat_obj_num],old_relat_obj_num)), (350 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             else:
    #                 cv2.putText(data_img, str("Relat Object: {0} | {1}".format(old_relat_obj_rank,"None")), (350 , 60), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if search_object != "None":
    #                 cv2.putText(data_img, str("Search list name: {0} {1}".format(search_list_name,search_list)), (350 , 80), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(data_img, str("Relat Object rank: {0} {1} {2} {3}".format(relat_obj_num ,relat_obj_rank,old_relat_obj_num,old_relat_obj_rank)), (350 , 160), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #                 cv2.putText(data_img, str("Searching | Track State: {0} {1}".format(searching_state ,track_state)), (350 , 140), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("fix_angle_state: {0} {1}".format(turn_state ,fix_angle_state)), (350 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.putText(data_img, str("To Arduino: {}".format(arduino_msg)), (350 , 200), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if start_angle != []:
    #                 cv2.putText(Left_RGB, str("{0:.1f}".format(start_angle)), (20 , 180), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             if found_obj !=[]:
    #                 cv2.putText(data_img, str("found object: {}".format(found_obj)), (650 , 160), cv2.FONT_HERSHEY_DUPLEX,0.5, (0, 255, 255), 1, cv2.LINE_AA)
    #             image = np.vstack((data_img,Left_RGB))
    #             cv2.imshow("Depth", Depth_GRAY)
    #             cv2.imshow("image", image)
    #         if cv2.waitKey(1) and data_raw == 'Q':
    #             break
            
    #     cv2.destroyAllWindows()

    #     cam.close()
    #     print("\nFINISH")
    #     time.sleep(1)


