#!/usr/bin/env python3


import math
import rospy
import numpy as np
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import Quaternion, Point
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

from config import Config
from gpd_interface import GPDInterface
from control.hardware_interface import HardwareInterface


class PickPlace(object):
    def __init__(self, move_group):
        super(PickPlace, self).__init__()
        self.config = Config()
        self.move_group = move_group
        self.gpd_interface = GPDInterface()
        self.hardware_interface = HardwareInterface(move_group)
        
    def pick(self, camera_pose):
        world_pose = self.gpd_interface.camera2world(
            camera_pose.pose.position,
            camera_pose.pose.orientation
        )
        
        grasp = self.set_grasp_pose(world_pose)
        self.open_gripper(grasp.pre_grasp_posture)
        self.close_gripper(grasp.grasp_posture)
        
        grasp_path = self.hardware_interface.get_grasp_trajectory(grasp)
        
        return grasp_path

    def place(self, camera_pose):
        world_pose = self.gpd_interface.camera2world(
            camera_pose.pose.position,
            camera_pose.pose.orientation
        )
        
        place = self.set_place_pose(world_pose)
        self.open_gripper(place.post_place_posture)
        
        place_path = self.hardware_interface.get_place_trajectory(place)
        
        return place_path
        
    def open_gripper(self, posture):
        posture.header.frame_id = self.config.base
        posture.joint_names = [self.config.left_finger, self.config.right_finger]
        point = JointTrajectoryPoint()
        point.positions = [0.04, 0.04]
        point.time_from_start = rospy.Duration(0.5)
        posture.points.append(point)
    
    def close_gripper(self, posture):
        posture.header.frame_id = self.config.base
        posture.joint_names = [self.config.left_finger, self.config.right_finger]
        point = JointTrajectoryPoint()
        point.positions = [0.02, 0.02]
        point.time_from_start = rospy.Duration(0.5)
        posture.points.append(point)
        
    def set_grasp_pose(self, world_pose):
        grasp = Grasp()
        grasp.grasp_pose.header.frame_id = self.config.base
        
        # pre_grasp_direction = quaternion_matrix([
        #     world_pose.pose.orientation.w,
        #     world_pose.pose.orientation.x,
        #     world_pose.pose.orientation.y,
        #     world_pose.pose.orientation.z
        # ])[:3, :3]
        # ratio = math.sqrt(
        #     pre_grasp_direction[2][0]*pre_grasp_direction[2][0]+\
        #     pre_grasp_direction[2][1]*pre_grasp_direction[2][1]+\
        #     pre_grasp_direction[2][2]*pre_grasp_direction[2][2]
        # )
        # grasp.grasp_pose.pose.position = Point(
        #     world_pose.pose.position.x-0.1*pre_grasp_direction[2][0]/(ratio),
        #     world_pose.pose.position.y-0.1*pre_grasp_direction[2][1]/(ratio),
        #     world_pose.pose.position.z-0.1*pre_grasp_direction[2][2]/(ratio)
        # )
        # grasp.grasp_pose.pose.orientation = world_pose.pose.orientation
        # grasp.pre_grasp_approach.direction.vector.x = pre_grasp_direction[2][0]
        # grasp.pre_grasp_approach.direction.vector.y = pre_grasp_direction[2][1]
        # grasp.pre_grasp_approach.direction.vector.z = pre_grasp_direction[2][2]
        
        grasp.grasp_pose.pose.position = Point(
            world_pose.pose.position.x-0.15,
            world_pose.pose.position.y,
            world_pose.pose.position.z
        )
        quaternion = quaternion_from_euler(0, -math.pi/2.5, 0)
        # quaternion = quaternion_from_matrix(world_pose.orientation)
        grasp.grasp_pose.pose.orientation = Quaternion(*quaternion)
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.direction.vector.z = -0.3
        
        grasp.pre_grasp_approach.desired_distance = 0.12
        grasp.pre_grasp_approach.min_distance = 0.1
        
        # grasp.post_grasp_retreat.direction.vector.z = 1.0
        # grasp.post_grasp_retreat.desired_distance = 0.01
        # grasp.post_grasp_retreat.min_distance = 0.0
        
        return grasp
    
    def set_place_pose(self, world_pose):
        place = PlaceLocation()
        place.place_pose.header.frame_id = self.config.base
        place.place_pose.pose.position = Point(
            world_pose.pose.position.x-0.15,
            world_pose.pose.position.y-0.2,
            world_pose.pose.position.z+0.1
        )
        
        quaternion = quaternion_from_euler(math.pi, 0, 0)
        place.place_pose.pose.orientation = Quaternion(*quaternion)
        # place.place_pose.pose.orientation.x = 1.0
        # place.place_pose.pose.orientation.y = 0
        # place.place_pose.pose.orientation.z = 0
        # place.place_pose.pose.orientation.w = 0
        
        place.pre_place_approach.direction.vector.z = -1.0
        place.pre_place_approach.desired_distance = 0.1
        place.pre_place_approach.min_distance = 0.08
        
        place.post_place_retreat.direction.vector.x = -1.0
        place.post_place_retreat.desired_distance = 0.1
        place.post_place_retreat.min_distance = 0.08
        
        return place
