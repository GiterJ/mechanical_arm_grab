#!/usr/bin/env python3


import rospy
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler

from config import Config


class PublishAxes(object):
    def __init__(self):
        super(PublishAxes, self).__init__()
        self.config = Config()
        self.thread_lock = True
    
    def publish_camera_axes(self):
        broadcaster = TransformBroadcaster()
        transform_stamped = TransformStamped()
        transform_stamped.header.frame_id = self.config.base
        transform_stamped.child_frame_id = self.config.camera_id
        
        transform_stamped.transform.translation.x = 0.99
        transform_stamped.transform.translation.y = 0.725
        transform_stamped.transform.translation.z = 1.19
        quaternion = quaternion_from_euler(-math.pi/2 - math.pi*45/180, 0, -math.pi)
        transform_stamped.transform.rotation = Quaternion(*quaternion)
        rate = rospy.Rate(10.0)

        # transform_stamped.transform.rotation.x = -0.4868437510025799
        # transform_stamped.transform.rotation.y = -0.6414227144104302
        # transform_stamped.transform.rotation.z = 0.48878027934155305
        # transform_stamped.transform.rotation.w = 0.33563954188219114
        
        while self.thread_lock:
            transform_stamped.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(transform_stamped)
            rate.sleep()
        