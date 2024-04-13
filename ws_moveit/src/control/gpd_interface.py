#!/usr/bin/env python3


import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_matrix
from tf import TransformListener
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from threading import Thread

from config import Config
from publish_axes import PublishAxes


class GPDInterface(object):
    def __init__(self):
        super(GPDInterface, self).__init__()
        self.config = Config()
    
    def camera2world(self, position, orientation):
        publish_axes = PublishAxes()
        axes_thread = Thread(target=publish_axes.publish_camera_axes)
        axes_thread.start()
        
        camera_pose = PoseStamped()
        camera_pose.header.stamp = rospy.Time.now()
        camera_pose.header.frame_id = self.config.camera_id
        camera_pose.pose.position = Point(position[0], position[1], position[2])
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = orientation
        homogeneous_matrix[:3, 3] = position
        quaternion = quaternion_from_matrix(np.array(homogeneous_matrix))
        camera_pose.pose.orientation = Quaternion(*quaternion)
        
        tf_buffer = Buffer(rospy.Duration(300.0))
        tf_listener = TransformListener(tf_buffer)
        
        while not tf_buffer.can_transform(
            self.config.base,
            self.config.camera_id,
            rospy.Time.now(),
            rospy.Duration(0.1)
        ): pass
        trans = tf_buffer.lookup_transform(
            self.config.base,
            self.config.camera_id,
            rospy.Time.now(),
            rospy.Duration(0.5)
        )
        world_pose = do_transform_pose(camera_pose, trans)
        publish_axes.thread_lock = False
        
        return world_pose
        