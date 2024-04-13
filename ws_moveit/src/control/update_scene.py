#!/usr/bin/env python3


import rospy
import numpy as np
import open3d as o3d
from geometry_msgs.msg import PoseStamped, Point
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs.point_cloud2 import create_cloud_xyz32

from config import Config
from gpd_interface import GPDInterface


class UpdateScene(object):
    def __init__(self, scene):
        super(UpdateScene, self).__init__()
        self.config = Config()
        self.scene = scene
        self.apply_scene_service = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
        self.planning_scene = PlanningScene()
        self.gpd_interface = GPDInterface()
        
    def update_scene(self, camera_pose):
        world_pose = self.gpd_interface.camera2world(
            camera_pose.pose.position,
            camera_pose.pose.orientation
        )
        
        table = CollisionObject()
        table.id = self.config.table_id
        table.header.frame_id = self.config.base
        
        table_primitive = SolidPrimitive()
        table_primitive.type = SolidPrimitive().BOX
        table_primitive.dimensions = [0.6, 1.0, world_pose.pose.position.z-0.14]
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.config.base
        table_pose.pose.position = Point(
            world_pose.pose.position.x,
            world_pose.pose.position.y,
            (world_pose.pose.position.z-0.14)/2
        )
        
        table.primitives = [table_primitive]
        table.primitive_poses = [table_pose.pose]
        self.planning_scene.world.collision_objects.append(table)
        
        
        target = CollisionObject()
        target.id = self.config.target_id
        target.header.frame_id = self.config.base
        
        target_primitive = SolidPrimitive()
        target_primitive.type = SolidPrimitive().CYLINDER
        target_primitive.dimensions = [0.28, 0.02]
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.config.base
        target_pose.pose.position = world_pose.pose.position
        # target_pose.pose.orientation = world_pose.pose.orientation
        print("                                        ==target position==")
        print(target_pose)
        
        target.primitives = [target_primitive]
        target.primitive_poses = [target_pose.pose]
        self.planning_scene.world.collision_objects.append(target)
        self.planning_scene.is_diff = True
        response = self.apply_scene_service(self.planning_scene)
    
    def update_point(self):
        publisher = rospy.Publisher(
            self.config.publish_point_cloud_topic,
            PointCloud2,
            queue_size=1
        )
        rate = rospy.Rate(10)
        
        cloud = o3d.io.read_point_cloud("/home/sakura/ws_moveit/src/demo_pkg/scripts/cloud/cloud.ply")
        points = np.asarray(cloud.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.config.base
        
        while not rospy.is_shutdown():
            point_cloud = create_cloud_xyz32(header, points)
            publisher.publish(point_cloud)
            rate.sleep()
        