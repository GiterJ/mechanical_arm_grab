#!/usr/bin/env python3
import sys
import rospy
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
sys.path.append('/home/ws_moveit/src/Diana7/bin')
import DianaApi as D

class RobotArmActionServer(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer("simple/follow_joint_trajectory", FollowJointTrajectoryAction, self.execute_cb, False)
        self._as.start()

        self.ip_addr = '192.168.10.75'
        self.netInfo=('192.168.10.75', 0, 0, 0, 0, 0)
        D.initSrv(self.netInfo, None , None)
    
    def execute_cb(self, goal):
        trajectory_points = goal.trajectory.points
        joint_names = goal.trajectory.joint_names
        joint_positions = []
        vel=0.1
        acc=0.05

        ret = D.createComplexPath(D.complex_path_type.NORMAL_JOINT_PATH, self.ip_addr)
        if ret[0]==0:
            for point in trajectory_points:
                D.addMoveJSegmentByTarget(ret[1],point.positions, vel, acc, 0.1, self.ip_addr)
            rospy.loginfo("Running complex path....")
            D.runComplexPath(ret[1], self.ip_addr)
            D.wait_move()
            rospy.loginfo("Run complex path successfully!")
        D.destroyComplexPath(ret[1])

        result = FollowJointTrajectoryResult()
        result.error_code = result.SUCCESSFUL
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('diana_action_server')
    server = RobotArmActionServer()
    rospy.spin()