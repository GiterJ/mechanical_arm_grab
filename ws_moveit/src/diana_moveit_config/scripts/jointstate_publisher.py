#!/usr/bin/env python3
import time
import sys
sys.path.append('/home/ws_moveit/src/Diana7/bin')
import DianaApi as D
import rospy
from sensor_msgs.msg import JointState



def joint_states_publisher():
    ip_addr = '192.168.10.75'
    def errorCallback(e):
        print("error code" + str(e))
    def robotStateCallback(stateInfo):
        for i in range(0,7):
            print(stateInfo.contents.jointPos(i))
        for i in range(0,7):
            print(stateInfo.contents. jointAngularVel(i))
    fnError = D.FNCERRORCALLBACK(errorCallback)
    fnState = D.FNCSTATECALLBACK(robotStateCallback)
    netInfo=('192.168.10.75', 0, 0, 0, 0, 0)
    D.initSrv(netInfo, fnError, None)


    joints_pos = [0,0,0,0,0,0,0]
    joints_vel = [0,0,0,0,0,0,0]
    joints_tor = [0,0,0,0,0,0,0]
    

    rospy.init_node("joint_state_publisher")
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        D.getJointPos(joints_pos, ip_addr)
        D.getJointAngularVel(joints_vel, ip_addr)
        D.getJointTorque(joints_tor, ip_addr)

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        joint_state.position = joints_pos
        joint_state.velocity = joints_vel
        joint_state.effort = joints_tor

        print(joint_state)
        pub.publish(joint_state)

        rate.sleep()


if __name__ == "__main__":
    joint_states_publisher()