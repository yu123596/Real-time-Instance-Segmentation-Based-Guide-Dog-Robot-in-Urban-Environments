# -*- coding: utf-8 -*-
"""
Created on Tue May  2 17:17:23 2023

@author: 309-1-RTX3060
"""


def camera_setting():
    import pyzed.sl as sl

    camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
    str_camera_settings = "BRIGHTNESS"
    step_camera_settings = 1

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

    print_camera_information(cam)
    print_help()
    
def start_camera():
    