#!/usr/bin/env python3

import time
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import cv2
from math import sqrt
import subprocess
import sys
sys.path.append("/home/ws_moveit/src/yolov7_mask")
sys.path.append("/home/ws_moveit/src/Diana7/bin")
sys.path.append("/home/ws_moveit/src/control")
from yolov7_mask.Detector import Detector
from Diana7.Actuator_claw import Actuator
from control.controller import Controller
import DianaApi as D

if __name__ == "__main__":

    # 创建detector对象
    detector = Detector(width=848,height=480,cx=427.5453796386719,cy = 236.88723754882812,
                        fx=425.6839904785156,fy = 425.6839904785156, debug=False)
    controller = Controller()
    actuator = Actuator()
    # 获取rgb图和深度图
    rgb, depth = detector.get_picture_from_flow()

    # 调用get_position()，获取物体位置
    flag_target, img_label, target_position = detector.get_target_position(rgb=rgb, depth=depth, target="bottle")
    
    # 如果识别到了要抓取的物体，继续执行
    if flag_target:
        gpd_file_path = "/home/ws_moveit/src/gpd"
        flag_grasp, grasp_position, grasp_orientation = detector.get_grasp_pose(rgb, depth, gpd_file_path, target_position)
        
        # 抓取或输出提示信息
        if flag_grasp:
            print("Grasp the target in {} with orientation {}".format(grasp_position, grasp_orientation))
            # input("initializing...")
            controller.update(target_position=target_position, target_orientation=np.zeros((3,3)))
            time.sleep(0.2)
            # input("pick...")
            actuator.claw_open()
            grasp_path = controller.pick(grasp_position=grasp_position, grasp_orientation=grasp_orientation)
            actuator.run_complex_path(grasp_path, vel=0.03)
            actuator.claw_close()
            controller.move_group.attach_object("target", "diana_hand")
            actuator.pull_place()
            # actuator.claw_open()
            # input("place...")
            # place_path = controller.place(grasp_position=grasp_position, grasp_orientation=grasp_orientation)
            # actuator.run_complex_path(place_path, vel=0.03)
            # actuator.claw_open()

    # 关闭视频流和通信服务
    detector.close_detector()
    actuator.close_actuator()
    


"""
Processes:
    1.create detector object
    2.get the rgb&depth image
    3.get target's position
    4.generate the point cloud
    5.predict the grasp position
    6.execute grasping
"""
