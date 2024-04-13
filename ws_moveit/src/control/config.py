#!/usr/bin/env python3


import numpy as np


class Config(object):
    def __init__(self):
        super(Config, self).__init__()
        # arm config
        self.node_name = "arm_grasp"
        self.group_name = "diana_arm"
        # self.group_name = "panda_arm"
        
        # topic config
        self.display_trajectory_topic = "/move_group/display_planned_path"
        self.publish_point_cloud_topic = "/point_cloud"
        
        # joint config
        self.base = "base_link"
        self.left_finger = "diana_finger_joint1"
        self.right_finger = "diana_finger_joint2"
        # self.base = "panda_link0"
        # self.left_finger = "panda_finger_joint1"
        # self.right_finger = "panda_finger_joint2"
        
        # scene object id
        self.table_id = "table"
        self.target_id = "target"
        self.camera_id = "camera"
        self.grasp_id = "grasp"
        
        # grasp and target argv
        self.grasp_position = \
            [-0.0283825, -0.0633246, 0.433403]
        self.grasp_orientation = \
            [[0.812453, 0.103791, 0.573714], [-0.561706, -0.124329, 0.817942], [0.156224, -0.986798, -0.0427111]]
        self.target_position = \
            [-0.06710737538696977, -0.03580504723849351, 0.4618166333238068]
            
        