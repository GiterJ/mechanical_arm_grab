#!/usr/bin/env python3


import rospy
import numpy as np
from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory

from config import Config


class HardwareInterface(object):
    def __init__(self, move_group):
        super(HardwareInterface, self).__init__()
        self.config = Config()
        self.move_group = move_group
        self.path = []
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.get_trajectory)
        path_thread = Thread(target=self.span)
        path_thread.start()
    
    def get_grasp_trajectory(self, grasp):
        print("                                    ====origin grasp position====")
        print(grasp.grasp_pose)
        self.move_group.set_support_surface_name(self.config.table_id)
        x_threshold = np.arange(
            grasp.grasp_pose.pose.position.x+0.05,
            grasp.grasp_pose.pose.position.x-0.15,
            -0.01
        )
        for x in x_threshold:
            grasp.grasp_pose.pose.position.x = x
            result = self.move_group.pick(self.config.target_id, grasp, plan_only=True)
            if result == MoveItErrorCodes.SUCCESS:
                print("                                    ====found grasp position====")
                print(grasp.grasp_pose)
                break
        else:
            print("No path planning found")
            exit(0)
        return self.path
    
    def get_place_trajectory(self, place):
        self.move_group.set_support_surface_name(self.config.table_id)
        result = self.move_group.place(self.config.target_id, place)
        return self.path

    def get_trajectory(self, trajectory):
        self.path = []
        for robot_trajectory in trajectory.trajectory:
            points = robot_trajectory.joint_trajectory.points
            for point in points:
                if len(point.positions) == 7:
                    self.path.append(point.positions)
        
    def span(self):
        rospy.spin()

