#!/usr/bin/env python3


import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped

from config import Config
from pick_place import PickPlace
from update_scene import UpdateScene
from gpd_interface import GPDInterface


class Controller(object):
    def __init__(self):
        super(Controller, self).__init__()
        config = Config()
        self.config = config
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(config.node_name, anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(config.group_name)
        # move_group.set_goal_position_tolerance(0.05)
        # move_group.set_goal_orientation_tolerance(0.02)
        display_trajectory_publisher = rospy.Publisher(
            config.display_trajectory_topic,
            DisplayTrajectory,
            queue_size=20
        )
        pick_place = PickPlace(move_group)
        update_scene = UpdateScene(scene)
        gpd_interface = GPDInterface()
        
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.pick_place = pick_place
        self.update_scene = update_scene
        self.gpd_interface = gpd_interface
        
    def update(self, target_position, target_orientation):
        """
        target position
        """
        camera_pose = PoseStamped()
        camera_pose.pose.position = target_position
        camera_pose.pose.orientation = target_orientation
        self.update_scene.update_scene(camera_pose)
        # self.update_scene.add_point()
        pass
        
    def pick(self, grasp_position, grasp_orientation):
        """
        grasp point
        """
        camera_pose = PoseStamped()
        camera_pose.pose.position = grasp_position
        camera_pose.pose.orientation = grasp_orientation
        return self.pick_place.pick(camera_pose)
        
    def place(self, grasp_position, grasp_orientation):
        camera_pose = PoseStamped()
        camera_pose.pose.position = grasp_position
        camera_pose.pose.orientation = grasp_orientation
        return self.pick_place.place(camera_pose)
    