#!usr/bin/python3


from ctypes import *
import platform
import os
import time
import sys
from enum import Enum
import math

errorCodeMessage = {
    0: 'NO_ERROR_CODE',
    -1000: 'ERROR_CODE_SOCKET_OTHER_ERROR',
    -1001: 'ERROR_CODE_WSASTART_FAIL',
    -1002: 'ERROR_CODE_CREATE_SOCKET_FAIL',
    -1003: 'ERROR_CODE_BIND_PORT_FAIL',
    -1004: 'ERROR_CODE_SOCKET_READ_FAIL',
    -1005: 'ERROR_CODE_SOCKET_TIMEOUT',
    -1006: 'ERROR_CODE_RECVFROM_FAIL',
    -1007: 'ERROR_CODE_SENDTO_FAIL',
    -1008: 'ERROR_CODE_LOST_HEARTBEAT',
    -1009: 'ERROR_CODE_LOST_ROBOTSTATE',
    -1010: 'ERROR_CODE_GET_DH_FAILED',
    -1011: 'ERROR_CODE_RELEASE_BRAKE_FAILED',
    -1012: 'ERROR_CODE_HOLD_BRAKE_FAILED',
    -2001: 'ERROR_CODE_JOINT_REGIST_ERROR',
    -2101: 'ERROR_CODE_COMMUNICATE_ERROR',
    -2201: 'ERROR_CODE_CALLING_CONFLICT_ERR',
    -2202: 'ERROR_CODE_COLLISION_ERROR',
    -2203: 'ERROR_CODE_NOT_FOLLOW_POSITION_CMD',
    -2204: 'ERROR_CODE_NOT_FOLLOW_TCP_CMD',
    -2205: 'ERROR_CODE_NOT_ALL_AT_OP_STATE',
    -2206: 'ERROR_CODE_OUT_RANGE_FEEDBACK',
    -2207: 'ERROR_CODE_EMERGENCY_STOP',
    -2208: 'ERROR_CODE_NO_INIT_PARAMETER',
    -2209: 'ECODE_NOT_MATCH_LOAD',
    -2210: 'ECODE_CANNOT_MOVE_WHILE_FREE_DRIVING', 
    -2211: 'ECODE_CANNOT_MOVE_WHILE_ZERO_SPACE_FREE_DRIVING',
    -2212: 'ECODE_PASSTHROUGH_FIFO_OVERFLOW_UP_ERROR',
    -2213: 'ECODE_PASSTHROUGH_FIFO_OVERFLOW_DOWN_ERROR',
    -2214: 'ERROR_CODE_ROBOT_IN_VIRTUAL_WALL',
    -2215: 'ERROR_CODE_CONFLICT_TASK_RUNNING',
    -2301: 'ERROR_CODE_PLAN_ERROR',
    -2302: 'ERROR_CODE_INTERPOLATE_ERROR',
    -2303: 'ERROR_CODE_INTERPOLATE_TORQUE_ERROR',
    -2304: 'ERROR_CODE_SINGULAR_VALUE_ERROR',
    -3001: 'ERROR_CODE_RESOURCE_UNAVAILABLE',
    -3002: 'ERROR_CODE_DUMP_LOG_TIMEOUT',
    -3003: 'ERROR_CODE_DUMP_LOG_FAILED',
    -3004: 'RESET_DH_FAILED',
}

scriptLibraryNames = {
    'Linux': 'libDianaApi.so',
    'Windows': 'DianaApi.dll'
}

scriptLibraryDir = os.path.split(os.path.realpath(__file__))[0]
scriptLibraryPath = os.path.join(
    scriptLibraryDir, scriptLibraryNames[platform.system()])
api_mod = None
if sys.version_info < (3, 8):
    api_mod = CDLL(scriptLibraryPath)
else:
    api_mod = CDLL(scriptLibraryPath, winmode=0)

# Enumration starts
# add Enumration here


class tcp_direction_e(Enum):
    T_MOVE_X_UP = 0
    T_MOVE_X_DOWN = 1
    T_MOVE_Y_UP = 2
    T_MOVE_Y_DOWN = 3
    T_MOVE_Z_UP = 4
    T_MOVE_Z_DOWN = 5


class mode_e(Enum):
    T_MODE_INVALID = -1
    T_MODE_POSITION = 0
    T_MODE_JOINT_IMPEDANCE = 1
    T_MODE_CART_IMPEDANCE = 2


class joint_direction_e(Enum):
    T_MOVE_UP = 0
    T_MOVE_DOWN = 1


class complex_path_type(Enum):
    NORMAL_JOINT_PATH = 0
    MOVEP_JOINT_PATH = 1
    NORMAL_POSE_PATH = 2
    MOVEP_POSE_PATH = 3


class coordinate_e(Enum):
    E_BASE_COORDINATE = 0
    E_TOOL_COORDINATE = 1
    E_WORK_PIECE_COORDINATE = 2
    E_VIEW_COORDINATE = 3


class collision_level(Enum):
    E_NO_COLLISION_DETECTION = 0
    E_JOINT_SPACE_DETECTION = 1
    E_CART_SPACE_DETECTION = 2
    E_TCP_RESULTANT_DETECTION = 3


class zero_space_move_direction(Enum):
    E_FORWARD = 1
    E_BACKWARD = -1

# Enumeration ends

# Structure starts
# add Structure here


class DIANA_JOINTS_POSITION(Structure):
    pass


DIANA_JOINTS_POSITION._fields_ = [
    ('values', c_double*7)
]


class DIANA_JOINTS_FORCE(Structure):
    pass


DIANA_JOINTS_FORCE._fields_ = [
    ('values', c_double*7)
]


class DIANA_TCP_POSE(Structure):
    pass


DIANA_TCP_POSE._fields_ = [
    ('values', c_double * 6)
]


class DIANA_JOINTS_SPEED(Structure):
    pass


DIANA_JOINTS_SPEED._fields_ = [
    ('values', c_double*7)
]


class DIANA_JOINTS_SPEED_L(Structure):
    pass


DIANA_JOINTS_SPEED_L._fields_ = [
    ('values', c_double*6)
]


class DIANA_JOINTS_ACC(Structure):
    pass


DIANA_JOINTS_ACC._fields_ = [
    ('values', c_double*2)
]


class DIANA_FRAME_MATRIX(Structure):
    pass


DIANA_FRAME_MATRIX._fields_ = [
    ('values', c_double * 16)
]


class DIANA_FORCE_DIRECTION(Structure):
    pass


DIANA_FORCE_DIRECTION._fields_ = [
    ('values', c_double * 3)
]


class SRV_NET_ST(Structure):
    pass


SRV_NET_ST._fields_ = [
    ('ipAddress', c_char * 32),
    ('SLocHeartbeatPortrvIp', c_ushort),
    ('LocRobotStatePort', c_ushort),
    ('LocSrvPort', c_ushort),
    ('LocRealtimeSrvPort', c_ushort),
    ('LocPassThroughSrvPort', c_ushort)
]


class DIANA_JOINT_ANGULAR_VEL(Structure):
    pass


DIANA_JOINT_ANGULAR_VEL._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_ACC(Structure):
    pass


DIANA_JOINT_ACC._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_CURRENT(Structure):
    pass


DIANA_JOINT_CURRENT._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_TORQUE(Structure):
    pass


DIANA_JOINT_TORQUE._fields_ = [
    ('values', c_double * 7)
]


class DIANA_TCP_FORCE(Structure):
    pass


DIANA_TCP_FORCE._fields_ = [
    ('values', c_double * 6)
]


class DIANA_DEFAULT_TCP(Structure):
    pass


DIANA_DEFAULT_TCP._fields_ = [
    ('values', c_double * 16)
]


class DIANA_TCP_VECTOR(Structure):
    pass


DIANA_TCP_VECTOR._fields_ = [
    ('values', c_double * 6)
]


class DIANA_RPY_VECTOR(Structure):
    pass


DIANA_RPY_VECTOR._fields_ = [
    ('values', c_double * 3)
]


class DIANA_AXIS_VECTOR(Structure):
    pass


DIANA_AXIS_VECTOR._fields_ = [
    ('values', c_double * 3)
]


class DIANA_JOINT_COLLISION(Structure):
    pass


DIANA_JOINT_COLLISION._fields_ = [
    ('values', c_double * 7)
]


class DIANA_CART_COLLISION(Structure):
    pass


DIANA_CART_COLLISION._fields_ = [
    ('values', c_double * 6)
]


class DIANA_JOINT_STIFF(Structure):
    pass


DIANA_JOINT_STIFF._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_DAMP(Structure):
    pass


DIANA_JOINT_DAMP._fields_ = [
    ('values', c_double * 7)
]


class DIANA_CART_STIFF(Structure):
    pass


DIANA_CART_STIFF._fields_ = [
    ('values', c_double * 6)
]


class DIANA_CART_DAMP(Structure):
    pass


DIANA_CART_DAMP._fields_ = [
    ('values', c_double * 6)
]


class DIANA_TCP_PAYLOAD(Structure):
    pass


DIANA_TCP_PAYLOAD._fields_ = [
    ('values', c_double * 10)
]


class StrTrajectoryState(Structure):
    _pack_ = 1
    _fields_ = [
        ('taskId', c_int),
        ('segCount', c_int),
        ('segIndex', c_int),
        ('errorCode', c_int),
        ('isPaused', c_int),
        ('isFreeDriving', c_int),
        ('isZeroSpaceFreeDriving', c_int)
    ]


class StrErrorInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('errorId', c_int),
        ('errorType', c_int),
        ('errorCode', c_int),
        ('errorMsg', c_char * 64)
    ]


class StrRobotStateInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('jointPos', c_double * 7),
        ('jointAngularVel', c_double * 7),
        ('jointCurrent', c_double * 7),
        ('jointTorque', c_double * 7),
        ('tcpPos', c_double * 6),
        ('tcpExternalForce', c_double),
        ('bCollision', c_bool),
        ('bTcpForceValid', c_bool),
        ('tcpForce', c_double * 6),
        ('jointForce', c_double * 7),
        ('trajState', StrTrajectoryState),
        ('StrErrorInfo', StrErrorInfo)
    ]


class DIANA_DH_A(Structure):
    pass


DIANA_DH_A._fields_ = [
    ('values', c_double * 7)
]


class DIANA_DH_Alpha(Structure):
    pass


DIANA_DH_Alpha._fields_ = [
    ('values', c_double * 7)
]


class DIANA_DH_D(Structure):
    pass


DIANA_DH_D._fields_ = [
    ('values', c_double * 7)
]


class DIANA_DH_Theta(Structure):
    pass


DIANA_DH_Theta._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JACOBI_MATRIX(Structure):
    pass


DIANA_JACOBI_MATRIX._fields_ = [
    ('values', c_double * 42)
]


class DIANA_DIGTAL_VALUE_ALL(Structure):
    pass


DIANA_DIGTAL_VALUE_ALL._fields_ = [
    ('values', c_int * 8)
]


class DIANA_ANALOG_VALUE_ALL(Structure):
    pass


DIANA_ANALOG_VALUE_ALL._fields_ = [
    ('values', c_double * 2)
]


class DIANA_ANALOG_MODE(Structure):
    pass


DIANA_ANALOG_MODE._fields_ = [
    ('values', c_int * 2)
]


class DIANA_DH_TCP_MEAS(Structure):
    pass


DIANA_DH_TCP_MEAS._fields_ = [
    ('values', c_double * 3)
]


class DIANA_DH_JOINT_POS__MEAS(Structure):
    pass


DIANA_DH_JOINT_POS__MEAS._fields_ = [
    ('values', c_double * 7)
]


class DIANA_DH_R(Structure):
    pass


DIANA_DH_R._fields_ = [
    ('values', c_double * 28)
]


class DIANA_DH_WRT(Structure):
    pass


DIANA_DH_WRT._fields_ = [
    ('values', c_double * 6)
]


class DIANA_DH_TRT(Structure):
    pass


DIANA_DH_TRT._fields_ = [
    ('values', c_double * 3)
]


class DIANA_DH_CONFID(Structure):
    pass


DIANA_DH_CONFID._fields_ = [
    ('values', c_double * 2)
]


class DIANA_DH_RTw2b(Structure):
    pass


DIANA_DH_RTw2b._fields_ = [
    ('values', c_double * 6)
]


class DIANA_DH_RTf2t(Structure):
    pass


DIANA_DH_RTf2t._fields_ = [
    ('values', c_double * 3)
]


class DIANA_KP(Structure):
    pass


DIANA_KP._fields_ = [
    ('values', c_double * 7)
]


class DIANA_KD(Structure):
    pass


DIANA_KD._fields_ = [
    ('values', c_double * 7)
]


class DIANA_ACTIVE_TCP(Structure):
    pass


DIANA_ACTIVE_TCP._fields_ = [
    ('values', c_double * 6)
]

class DIANA_GRAV_INFO(Structure):
    pass


DIANA_GRAV_INFO._fields_ = [
    ('values', c_double * 3)
]

# Structure ends


# Callback function starts
# add callback function here
FNCERRORCALLBACK = CFUNCTYPE(None, c_int)

FNCSTATECALLBACK = CFUNCTYPE(None, POINTER(StrRobotStateInfo))
# Callback function ends


def message(ret):
    if(ret >= 0):
        return True
    else:
        errorcode = getLastError()
        errorMessage = errorCodeMessage.get(errorcode)
        callerName = 'function ' + sys._getframe().f_back.f_code.co_name + ' fails,'
        if errorMessage == None:
            print(callerName + 'Error Code ' + str(errorcode))
        else:
            print(callerName + errorMessage)
        return False


def wait_move():
    time.sleep(0.02)
    while True:
        state = getRobotState()
        if state != 0:
            break
        else:
            time.sleep(0.001)
    stop()


def initSrv(srv_net_st, fnError=None, fnState=None):
    pinfo = SRV_NET_ST()
    strIpAddress = srv_net_st[0]
    pinfo.ipAddress = bytes(strIpAddress.encode('utf-8'))
    pinfo.SLocHeartbeatPortrvIp = srv_net_st[1]
    pinfo.LocRobotStatePort = srv_net_st[2]
    pinfo.LocSrvPort = srv_net_st[3]
    pinfo.LocRealtimeSrvPort = srv_net_st[4]
    pinfo.LocPassThroughSrvPort = srv_net_st[5]
    api_mod.initSrv.argtypes = [FNCERRORCALLBACK,
                                FNCSTATECALLBACK, POINTER(SRV_NET_ST)]
    api_mod.initSrv.restype = c_int
    if fnError == None:
        fnError = FNCERRORCALLBACK()
    if fnState == None:
        fnState = FNCSTATECALLBACK()
    ret = api_mod.initSrv(fnError, fnState, byref(pinfo))
    return message(ret)


def destroySrv(ipAddress=''):
    api_mod.destroySrv.argtypes = [c_char_p]
    api_mod.destroySrv.restype = c_int
    ret = api_mod.destroySrv(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setPushPeriod(period, ipAddress=''):
    api_mod.setPushPeriod.argtypes = [c_int, c_char_p]
    api_mod.setPushPeriod.restype = c_int
    ret = api_mod.setPushPeriod(period, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveTCP(d, v, a, ipAddress=''):
    active_tcp = c_void_p(0)
    api_mod.moveTCP.argtypes = [c_int, c_double, c_double, c_void_p, c_char_p]
    api_mod.moveTCP.restype = c_int
    ret = api_mod.moveTCP(d.value, v, a, active_tcp,
                          bytes(ipAddress.encode('utf-8')))
    return message(ret)


def rotationTCP(d, v, a, ipAddress=''):
    active_tcp = c_void_p(0)
    api_mod.rotationTCP.argtypes = [
        c_int, c_double, c_double, c_void_p, c_char_p]
    api_mod.rotationTCP.restype = c_int
    ret = api_mod.rotationTCP(
        d.value, v, a, active_tcp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveJoint(d, i, v, a, ipAddress=''):
    api_mod.moveJoint.argtypes = [c_int, c_int, c_double, c_double, c_char_p]
    api_mod.moveJoint.restype = c_int
    ret = api_mod.moveJoint(d.value, i, v, a, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveJToTarget(joints, v, a, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.moveJToTarget.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double, c_char_p]
    api_mod.moveJToTarget.restype = c_int
    ret = api_mod.moveJToTarget(
        dianaJointPos, v, a, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveJToPose(pose, v, a, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    active_tcp = c_void_p(0)
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.moveJToPose.argtypes = [
        POINTER(DIANA_TCP_POSE), c_double, c_double, c_void_p, c_char_p]
    api_mod.moveJToPose.restype = c_int
    ret = api_mod.moveJToPose(
        dianaTcpPose, v, a, active_tcp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveLToTarget(joints, v, a, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.moveLToTarget.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double, c_char_p]
    api_mod.moveLToTarget.restype = c_int
    ret = api_mod.moveLToTarget(
        dianaJointPos, v, a, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveLToPose(pose, v, a, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    active_tcp = c_void_p(0)
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.moveLToPose.argtypes = [
        POINTER(DIANA_TCP_POSE), c_double, c_double, c_void_p, c_char_p]
    api_mod.moveLToPose.restype = c_int
    ret = api_mod.moveLToPose(
        dianaTcpPose, v, a, active_tcp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveJ(joints, v, a, ipAddress=''):
    ret = moveJToTarget(joints, v, a, ipAddress)
    return message(ret)


def moveL(pose, v, a, ipAddress=''):
    ret = moveLToPose(pose, v, a)
    return message(ret)


def speedJ(speed, acc, t=0.0, ipAddress=''):
    dianaJointsSpeed = DIANA_JOINTS_SPEED()
    dianaJointsSpeed.values = (c_double * 7)()
    for index in range(0, len(speed)):
        dianaJointsSpeed.values[index] = speed[index]
    api_mod.speedJ.argtypes = [
        POINTER(DIANA_JOINTS_SPEED), c_double, c_double, c_char_p]
    api_mod.speedJ.restype = c_int
    ret = api_mod.speedJ(dianaJointsSpeed, acc, t,
                         bytes(ipAddress.encode('utf-8')))
    return message(ret)


def speedL(speed, acc, t=0.0, ipAddress=''):
    dianaJointsSpeed = DIANA_JOINTS_SPEED_L()
    dianaJointsSpeed.values = (c_double * 6)()
    for index in range(0, len(dianaJointsSpeed.values)):
        dianaJointsSpeed.values[index] = speed[index]
    dianaJointsAcc = DIANA_JOINTS_ACC()
    dianaJointsAcc.values = (c_double * 2)()
    for index in range(0, len(dianaJointsAcc.values)):
        dianaJointsAcc.values[index] = acc[index]
    active_tcp = c_void_p(0)
    api_mod.speedL.argtypes = [POINTER(DIANA_JOINTS_SPEED_L), POINTER(
        DIANA_JOINTS_ACC), c_double, c_void_p, c_char_p]
    api_mod.speedL.restype = c_int
    ret = api_mod.speedL(dianaJointsSpeed, dianaJointsAcc,
                         t, active_tcp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def freeDriving(enable, ipAddress=''):
    api_mod.freeDriving.argtypes = [c_bool, c_char_p]
    api_mod.freeDriving.restype = c_int
    ret = api_mod.freeDriving(enable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def stop(ipAddress=''):
    api_mod.stop.argtypes = [c_char_p]
    api_mod.stop.restype = c_int
    ret = api_mod.stop(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def forward(joints, pose, ipAddress=''):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointsPosition.values[index] = joints[index]
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    active_tcp = c_void_p(0)
    api_mod.forward.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), POINTER(DIANA_TCP_POSE), c_void_p, c_char_p]
    api_mod.forward.restype = c_int
    ret = api_mod.forward(dianaJointsPosition, byref(
        dianaTcpPose), active_tcp, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(dianaTcpPose.values)):
        pose[index] = dianaTcpPose.values[index]
    return message(ret)


def inverse(pose, joints, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    active_tcp = c_void_p(0)
    api_mod.inverse.argtypes = [POINTER(DIANA_TCP_POSE), POINTER(
        DIANA_JOINTS_POSITION),  c_void_p, c_char_p]
    api_mod.inverse.restype = c_int
    ret = api_mod.inverse(dianaTcpPose, byref(
        dianaJointsPosition), active_tcp, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(joints)):
        joints[index] = dianaJointsPosition.values[index]
    return message(ret)


def getJointPos(jointsPosition, ipAddress=''):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    api_mod.getJointPos.argtypes = [POINTER(DIANA_JOINTS_POSITION), c_char_p]
    api_mod.getJointPos.restype = c_int
    ret = api_mod.getJointPos(
        byref(dianaJointsPosition), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(jointsPosition)):
        jointsPosition[index] = dianaJointsPosition.values[index]
    return message(ret)


def getJointAngularVel(jointAngularVel, ipAddress=''):
    dianaJointAngularVel = DIANA_JOINT_ANGULAR_VEL()
    dianaJointAngularVel.values = (c_double * 7)()
    api_mod.getJointAngularVel.argtypes = [
        POINTER(DIANA_JOINT_ANGULAR_VEL), c_char_p]
    api_mod.getJointAngularVel.restype = c_int
    ret = api_mod.getJointAngularVel(
        byref(dianaJointAngularVel), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(jointAngularVel)):
        jointAngularVel[index] = dianaJointAngularVel.values[index]
    return message(ret)


def getJointCurrent(jointCurrent, ipAddress=''):
    dianaJointCurrent = DIANA_JOINT_CURRENT()
    dianaJointCurrent.values = (c_double*7)()
    for index in range(0, len(jointCurrent)):
        dianaJointCurrent.values[index] = jointCurrent[index]
    api_mod.getJointCurrent.argtypes = [POINTER(DIANA_JOINT_CURRENT), c_char_p]
    api_mod.getJointCurrent.restype = c_int
    ret = api_mod.getJointCurrent(
        byref(dianaJointCurrent), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(jointCurrent)):
        jointCurrent[index] = dianaJointCurrent.values[index]
    return message(ret)


def getJointTorque(jointTorque, ipAddress=''):
    dianaJointTorque = DIANA_JOINT_TORQUE()
    dianaJointTorque.values = (c_double*7)()
    for index in range(0, len(jointTorque)):
        dianaJointTorque.values[index] = jointTorque[index]
    api_mod.getJointTorque.argtypes = [POINTER(DIANA_JOINT_TORQUE), c_char_p]
    api_mod.getJointTorque.restype = c_int
    ret = api_mod.getJointTorque(
        byref(dianaJointTorque), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(jointTorque)):
        jointTorque[index] = dianaJointTorque.values[index]
    return message(ret)


def getTcpPos(tcpPose, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    api_mod.getTcpPos.argtypes = [POINTER(DIANA_TCP_POSE), c_char_p]
    api_mod.getTcpPos.restype = c_int
    ret = api_mod.getTcpPos(byref(dianaTcpPose),
                            bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(dianaTcpPose.values)):
        tcpPose[index] = dianaTcpPose.values[index]
    return message(ret)


def getTcpExternalForce(ipAddress=''):
    api_mod.getTcpExternalForce.restype = c_double
    ret = api_mod.getTcpExternalForce(bytes(ipAddress.encode('utf-8')))
    return ret


def releaseBrake(ipAddress=''):
    api_mod.releaseBrake.argtypes = [c_char_p]
    api_mod.releaseBrake.restype = c_int
    ret = api_mod.releaseBrake(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def holdBrake(ipAddress=''):
    api_mod.holdBrake.argtypes = [c_char_p]
    api_mod.holdBrake.restype = c_int
    ret = api_mod.holdBrake(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def changeControlMode(m, ipAddress=''):
    api_mod.changeControlMode.argtypes = [c_int, c_char_p]
    api_mod.changeControlMode.restype = c_int
    ret = api_mod.changeControlMode(m.value, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def getLibraryVersion():
    api_mod.getLibraryVersion.restype = c_uint
    ret = api_mod.getLibraryVersion()
    return ret


def formatError(e, ipAddress=''):
    api_mod.formatError.argtypes = [c_int, c_char_p]
    api_mod.formatError.restype = c_char_p
    ret = api_mod.formatError(e, bytes(ipAddress.encode('utf-8')))
    return ret


def getLastError(ipAddress=''):
    api_mod.getLastError.argtypes = [c_char_p]
    api_mod.getLastError.restype = c_int
    ret = api_mod.getLastError(bytes(ipAddress.encode('utf-8')))
    return ret


def setLastError(e, ipAddress=''):
    api_mod.setLastError.argtypes = [c_int, c_char_p]
    api_mod.setLastError.restype = c_int
    ret = api_mod.setLastError(e, bytes(ipAddress.encode('utf-8')))
    return ret


def setDefaultActiveTcp(default_tcp, ipAddress=''):
    defaultTcp = DIANA_DEFAULT_TCP()
    defaultTcp.values = (c_double*16)()
    for index in range(0, len(defaultTcp.values)):
        defaultTcp.values[index] = default_tcp[index]
    api_mod.setDefaultActiveTcp.argtypes = [
        POINTER(DIANA_DEFAULT_TCP), c_char_p]
    api_mod.setDefaultActiveTcp.restype = c_int
    ret = api_mod.setDefaultActiveTcp(
        defaultTcp, bytes(ipAddress.encode('utf-8')))
    if(ret == 0):
        print("setDefaultActiveTcp succeeds")
        return True
    else:
        print("setDefaultActiveTcp fails")
        return False


def getLinkState(ipAddress=''):
    api_mod.getLinkState.argtypes = [c_char_p]
    api_mod.getLinkState.restype = c_int
    ret = api_mod.getLinkState(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def getTcpForce(tcpForce, ipAddress=''):
    dianaTcpForce = DIANA_TCP_FORCE()
    dianaTcpForce.values = (c_double * 6)()
    api_mod.getTcpForce.argtypes = [POINTER(DIANA_TCP_FORCE), c_char_p]
    api_mod.getTcpForce.restype = c_int
    ret = api_mod.getTcpForce(byref(dianaTcpForce),
                              bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(dianaTcpForce.values)):
        tcpForce[index] = dianaTcpForce.values[index]
    return message(ret)


def getJointForce(jointForce, ipAddress=''):
    dianaJointForce = DIANA_JOINTS_FORCE()
    dianaJointForce.values = (c_double * 7)()
    api_mod.getJointForce.argtypes = [POINTER(DIANA_JOINTS_FORCE), c_char_p]
    api_mod.getJointForce.restype = c_int
    ret = api_mod.getJointForce(
        byref(dianaJointForce), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(jointForce)):
        jointForce[index] = dianaJointForce.values[index]
    return message(ret)


def isCollision(ipAddress=''):
    api_mod.isCollision.argtypes = [c_char_p]
    api_mod.isCollision.restype = c_bool
    ret = api_mod.isCollision(bytes(ipAddress.encode('utf-8')))
    return ret


######################DH参数标定#############################


def initDHCali(tcpMeas, jntPosMeas, nrSets, ipAddress=''):
    DHtcpMeas = DIANA_DH_TCP_MEAS()
    DHtcpMeas.values = (c_double*3)()
    DHjntPosMeas = DIANA_DH_JOINT_POS__MEAS()
    DHjntPosMeas.values = (c_double*7)()
    for index in range(0, len(DHtcpMeas.values)):
        DHtcpMeas.values[index] = tcpMeas[index]
    for index in range(0, len(DHjntPosMeas.values)):
        DHjntPosMeas.values[index] = jntPosMeas[index]
    api_mod.initDHCali.restype = c_int
    api_mod.initDHCali.argtypes = [
        POINTER(DIANA_DH_TCP_MEAS), POINTER(DIANA_DH_JOINT_POS__MEAS), c_uint, c_char_p]
    ret = api_mod.initDHCali(byref(DHtcpMeas), byref(
        DHjntPosMeas), nrSets, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def getDHCaliResult(rDH, wRT, tRT, confid, ipAddress=''):
    dhR = DIANA_DH_R()
    dhR.values = (c_double*28)()
    rtW = DIANA_DH_WRT()
    rtW.values = (c_double*6)()
    rtT = DIANA_DH_TRT()
    rtT.values = (c_double*3)()
    dhConfid = DIANA_DH_CONFID()
    dhConfid.values = (c_double*2)()
    api_mod.getDHCaliResult.restype = c_int
    api_mod.getDHCaliResult.argtypes = [POINTER(DIANA_DH_R), POINTER(
        DIANA_DH_WRT), POINTER(DIANA_DH_TRT), POINTER(DIANA_DH_CONFID), c_char_p]
    ret = api_mod.getDHCaliResult(
        byref(dhR), byref(rtW), byref(rtT), byref(dhConfid), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(rDH)):
        rDH[index] = dhR.values[index]
    for index in range(0, len(wRT)):
        wRT[index] = rtW.values[index]
    for index in range(0, len(tRT)):
        tRT[index] = rtT.values[index]
    for index in range(0, len(confid)):
        confid[index] = dhConfid.values[index]
    return message(ret)
#_________________________________________________________________________________________#


def setDH(a, alpha, d, theta, ipAddress=''):
    arr_aDH = DIANA_DH_A()
    arr_aDH.values = (c_double*7)()
    for index in range(0, len(a)):
        arr_aDH.values[index] = a[index]
    arr_alphaDH = DIANA_DH_Alpha()
    arr_alphaDH.values = (c_double*7)()
    for index in range(0, len(alpha)):
        arr_alphaDH.values[index] = alpha[index]
    arr_dDH = DIANA_DH_D()
    arr_dDH.values = (c_double*7)()
    for index in range(0, len(d)):
        arr_dDH.values[index] = d[index]
    arr_thetaDH = DIANA_DH_Theta()
    arr_thetaDH.values = (c_double*7)()
    for index in range(0, len(theta)):
        arr_thetaDH.values[index] = theta[index]
    api_mod.setDH.restype = c_int
    api_mod.setDH.argtypes = [POINTER(DIANA_DH_A),
                              POINTER(DIANA_DH_Alpha), POINTER(DIANA_DH_D), POINTER(DIANA_DH_Theta), c_char_p]
    ret = api_mod.setDH(byref(arr_aDH), byref(arr_alphaDH),
                        byref(arr_dDH), byref(arr_thetaDH), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setWrd2BasRT(RTw2b, ipAddress=''):
    arr_RTw2b = DIANA_DH_RTw2b()
    arr_RTw2b.values = (c_double*6)()
    for index in range(0, len(RTw2b)):
        arr_RTw2b.values[index] = RTw2b[index]
    api_mod.setWrd2BasRT.restype = c_int
    api_mod.setWrd2BasRT.argtypes = [POINTER(DIANA_DH_RTw2b), c_char_p]
    ret = api_mod.setWrd2BasRT(
        byref(arr_RTw2b), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setFla2TcpRT(RTf2t, ipAddress=''):
    arr_RTf2t = DIANA_DH_RTf2t()
    arr_RTf2t.values = (c_double*3)()
    for index in range(0, len(RTf2t)):
        arr_RTf2t.values[index] = RTf2t[index]
    api_mod.setFla2TcpRT.restype = c_int
    api_mod.setFla2TcpRT.argtypes = [POINTER(DIANA_DH_RTf2t), c_char_p]
    ret = api_mod.setFla2TcpRT(
        byref(arr_RTf2t), bytes(ipAddress.encode('utf-8')))
    return message(ret)


#########################################################


def getRobotState(ipAddress=''):
    api_mod.getRobotState.argtypes = [c_char_p]
    api_mod.getRobotState.restype = c_int8
    ret = api_mod.getRobotState(bytes(ipAddress.encode('utf-8')))
    return ret


##########v2.0##########################


def resume(ipAddress=''):
    api_mod.resume.argtypes = [c_char_p]
    api_mod.resume.restype = c_int
    ret = api_mod.resume(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setJointCollision(collision, ipAddress=''):
    jointCollision = DIANA_JOINT_COLLISION()
    jointCollision.values = (c_double*7)()
    for index in range(0, len(collision)):
        jointCollision.values[index] = collision[index]
    api_mod.setJointCollision.argtypes = [
        POINTER(DIANA_JOINT_COLLISION), c_char_p]
    api_mod.setJointCollision.restype = c_int
    ret = api_mod.setJointCollision(
        jointCollision, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setCartCollision(collision, ipAddress=''):
    cartCollision = DIANA_CART_COLLISION()
    cartCollision.values = (c_double*6)()
    for index in range(0, len(cartCollision.values)):
        cartCollision.values[index] = collision[index]
    api_mod.setCartCollision.argtypes = [
        POINTER(DIANA_CART_COLLISION), c_char_p]
    api_mod.setCartCollision.restype = c_int
    ret = api_mod.setCartCollision(
        cartCollision, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def enterForceMode(frame_type, frame_matrix, force_direction, force_value, max_approach_velocity, max_allow_tcp_offset, ipAddress=''):
    dianaFrameMatrix = DIANA_FRAME_MATRIX()
    dianaFrameMatrix.values = (c_double * 16)()
    for index in range(0, len(dianaFrameMatrix.values)):
        dianaFrameMatrix.values[index] = frame_matrix[index]
    dianaForceDirection = DIANA_FORCE_DIRECTION()
    dianaForceDirection.values = (c_double * 3)()
    for index in range(0, len(dianaForceDirection.values)):
        dianaForceDirection.values[index] = force_direction[index]
    api_mod.enterForceMode.argtypes = [c_int, POINTER(DIANA_FRAME_MATRIX), POINTER(
        DIANA_FORCE_DIRECTION), c_double, c_double, c_double, c_char_p]
    api_mod.enterForceMode.restype = c_int
    ret = api_mod.enterForceMode(frame_type, byref(dianaFrameMatrix), byref(
        dianaForceDirection), force_value, max_approach_velocity, max_allow_tcp_offset, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def leaveForceMode(mode, ipAddress=''):
    if not isinstance(mode, mode_e):
        raise TypeError(
            'mode must be an instance of mode_e Enum')
    api_mod.leaveForceMode.argtypes = [c_int, c_char_p]
    api_mod.leaveForceMode.restype = c_int
    ret = api_mod.leaveForceMode(mode.value, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setDefaultActiveTcpPose(arrPose, ipAddress=''):
    tcpPose = DIANA_TCP_VECTOR()
    tcpPose.values = (c_double*6)()
    for index in range(0, len(tcpPose.values)):
        tcpPose.values[index] = arrPose[index]
    api_mod.setDefaultActiveTcpPose.argtypes = [
        POINTER(DIANA_TCP_VECTOR), c_char_p]
    api_mod.setDefaultActiveTcpPose.restype = c_int
    ret = api_mod.setDefaultActiveTcpPose(
        tcpPose, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setResultantCollision(force, ipAddress=''):
    api_mod.setResultantCollision.argtypes = [c_double, c_char_p]
    api_mod.setResultantCollision.restype = c_int
    ret = api_mod.setResultantCollision(
        force, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setJointImpedance(arrStiff, arrDamp, ipAddress=''):
    jointStiff = DIANA_JOINT_STIFF()
    jointStiff.values = (c_double*7)()
    for index in range(0, len(arrStiff)):
        jointStiff.values[index] = arrStiff[index]
    jointDamp = DIANA_JOINT_DAMP()
    jointDamp.values = (c_double*7)()
    for index in range(0, len(arrDamp)):
        jointDamp.values[index] = arrDamp[index]
    api_mod.setJointImpedance.argtypes = [
        POINTER(DIANA_JOINT_STIFF), POINTER(DIANA_JOINT_DAMP), c_char_p]
    api_mod.setJointImpedance.restype = c_int
    ret = api_mod.setJointImpedance(
        jointStiff, jointDamp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def getJointImpedance(arrStiff, arrDamp, ipAddress=''):
    jointStiff = DIANA_JOINT_STIFF()
    jointStiff.values = (c_double*7)()
    jointDamp = DIANA_JOINT_DAMP()
    jointDamp.values = (c_double*7)()
    api_mod.getJointImpedance.argtypes = [
        POINTER(DIANA_JOINT_STIFF), POINTER(DIANA_JOINT_DAMP), c_char_p]
    api_mod.getJointImpedance.restype = c_int
    ret = api_mod.getJointImpedance(byref(jointStiff), byref(
        jointDamp), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(arrStiff)):
        arrStiff[index] = jointStiff.values[index]
    for index in range(0, len(arrDamp)):
        arrDamp[index] = jointDamp.values[index]
    return message(ret)


def setCartImpedance(arrStiff, arrDamp, ipAddress=''):
    cartStiff = DIANA_CART_STIFF()
    cartStiff.values = (c_double*6)()
    for index in range(0, len(cartStiff.values)):
        cartStiff.values[index] = arrStiff[index]
    cartDamp = DIANA_CART_DAMP()
    cartDamp.values = (c_double*6)()
    for index in range(0, len(cartDamp.values)):
        cartDamp.values[index] = arrDamp[index]
    api_mod.setCartImpedance.argtypes = [
        POINTER(DIANA_CART_STIFF), POINTER(DIANA_CART_DAMP), c_char_p]
    api_mod.setCartImpedance.restype = c_int
    ret = api_mod.setCartImpedance(
        cartStiff, cartDamp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def getCartImpedance(arrStiff, arrDamp, ipAddress=''):
    cartStiff = DIANA_CART_STIFF()
    cartStiff.values = (c_double*6)()
    cartDamp = DIANA_CART_DAMP()
    cartDamp.values = (c_double*6)()
    api_mod.getCartImpedance.argtypes = [
        POINTER(DIANA_CART_STIFF), POINTER(DIANA_CART_DAMP), c_char_p]
    api_mod.getCartImpedance.restype = c_int
    ret = api_mod.getCartImpedance(byref(cartStiff), byref(
        cartDamp), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(cartStiff.values)):
        arrStiff[index] = cartStiff.values[index]
    for index in range(0, len(cartDamp.values)):
        arrDamp[index] = cartDamp.values[index]
    return message(ret)


def zeroSpaceFreeDriving(enable, ipAddress=''):
    api_mod.zeroSpaceFreeDriving.argtypes = [c_bool, c_char_p]
    api_mod.zeroSpaceFreeDriving.restype = c_int
    ret = api_mod.zeroSpaceFreeDriving(
        enable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def createPath(id_type, ipAddress=''):
    id_path = c_uint()
    api_mod.createPath.argtypes = [c_int, POINTER(c_uint), c_char_p]
    api_mod.createPath.restype = c_int
    ret = api_mod.createPath(id_type, byref(
        id_path), bytes(ipAddress.encode('utf-8')))
    return ret, id_path


def addMoveL(id_path, joints, vel, acc, blendradius, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveL.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_char_p]
    api_mod.addMoveL.restype = c_int
    ret = api_mod.addMoveL(id_path, dianaJointPos, vel,
                           acc, blendradius, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addMoveJ(id_path, joints, vel, acc, blendradius, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveJ.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_char_p]
    api_mod.addMoveJ.restype = c_int
    ret = api_mod.addMoveJ(id_path, dianaJointPos, vel,
                           acc, blendradius, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def runPath(id_path, ipAddress=''):
    api_mod.runPath.argtypes = [c_uint, c_char_p]
    api_mod.runPath.restype = c_int
    ret = api_mod.runPath(id_path, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def destroyPath(id_path, ipAddress=''):
    api_mod.destroyPath.argtypes = [c_uint, c_char_p]
    api_mod.destroyPath.restype = c_int
    ret = api_mod.destroyPath(id_path, bytes(ipAddress.encode('utf-8')))
    return message(ret)

###################### version 0.7#########################


def rpy2Axis(arr):
    _rpy = DIANA_RPY_VECTOR()
    _rpy.values = (c_double*3)()
    for index in range(0, len(_rpy.values)):
        _rpy.values[index] = arr[index]
    api_mod.rpy2Axis.argtypes = [POINTER(DIANA_RPY_VECTOR)]
    api_mod.rpy2Axis.restype = c_int
    ret = api_mod.rpy2Axis(byref(_rpy))
    for index in range(0, len(_rpy.values)):
        arr[index] = _rpy.values[index]
    return message(ret)


def axis2RPY(arr):
    _axis = DIANA_AXIS_VECTOR()
    _axis.values = (c_double*3)()
    for index in range(0, len(_axis.values)):
        _axis.values[index] = arr[index]
    api_mod.axis2RPY.argtypes = [POINTER(DIANA_AXIS_VECTOR)]
    api_mod.axis2RPY.restype = c_int
    ret = api_mod.axis2RPY(byref(_axis))
    for index in range(0, len(_axis.values)):
        arr[index] = _axis.values[index]
    return message(ret)


def homogeneous2Pose(matrix, pose):
    _matrix = DIANA_FRAME_MATRIX()
    _matrix.values = (c_double*16)()
    for index in range(0, len(_matrix.values)):
        _matrix.values[index] = matrix[index]
    _pose = DIANA_TCP_POSE()
    _pose.values = (c_double*6)()
    api_mod.homogeneous2Pose.argtypes = [
        POINTER(DIANA_FRAME_MATRIX), POINTER(DIANA_TCP_POSE)]
    api_mod.homogeneous2Pose.restype = c_int
    ret = api_mod.homogeneous2Pose(_matrix, byref(_pose))
    for index in range(0, len(_pose.values)):
        pose[index] = _pose.values[index]
    return message(ret)


def pose2Homogeneous(pose, matrix):
    _pose = DIANA_TCP_POSE()
    _pose.values = (c_double*6)()
    for index in range(0, len(_pose.values)):
        _pose.values[index] = pose[index]
    _matrix = DIANA_FRAME_MATRIX()
    _matrix.values = (c_double*16)()
    api_mod.pose2Homogeneous.argtypes = [
        POINTER(DIANA_TCP_POSE), POINTER(DIANA_FRAME_MATRIX)]
    api_mod.pose2Homogeneous.restype = c_int
    ret = api_mod.pose2Homogeneous(_pose, byref(_matrix))
    for index in range(0, len(_matrix.values)):
        matrix[index] = _matrix.values[index]
    return message(ret)


def enableTorqueReceiver(bEnable, ipAddress=''):
    api_mod.enableTorqueReceiver.restype = c_int
    api_mod.enableTorqueReceiver.argtypes = [c_bool, c_char_p]
    ret = api_mod.enableTorqueReceiver(
        bEnable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def sendTorque_rt(torque, t, ipAddress=''):
    jnt_torque = DIANA_JOINT_TORQUE()
    jnt_torque.values = (c_double*7)()
    for index in range(0, len(torque)):
        jnt_torque.values[index] = torque[index]
    api_mod.sendTorque_rt.restype = c_int
    api_mod.sendTorque_rt.argtypes = [
        POINTER(DIANA_JOINT_TORQUE), c_double, c_char_p]
    ret = api_mod.sendTorque_rt(
        byref(jnt_torque), t, bytes(ipAddress.encode('utf-8')))
    return message(ret)

###################### version 0.8#########################


def enableCollisionDetection(enable, ipAddress=''):
    api_mod.enableCollisionDetection.argtypes = [c_bool, c_char_p]
    api_mod.enableCollisionDetection.restype = c_int
    ret = api_mod.enableCollisionDetection(
        enable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setActiveTcpPayload(payload, ipAddress=''):
    tcpPayload = DIANA_TCP_PAYLOAD()
    tcpPayload.values = (c_double*10)()
    for index in range(0, len(tcpPayload.values)):
        tcpPayload.values[index] = payload[index]
    api_mod.setActiveTcpPayload.argtypes = [
        POINTER(DIANA_TCP_PAYLOAD), c_char_p]
    api_mod.setActiveTcpPayload.restype = c_int
    ret = api_mod.setActiveTcpPayload(
        tcpPayload, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def servoJ(joints_pos, t=0.01, ah_t=0.01, gain=300, ipAddress=''):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    for index in range(0, len(joints_pos)):
        dianaJointsPosition.values[index] = joints_pos[index]
    api_mod.servoJ.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_char_p]
    api_mod.servoJ.restype = c_int
    ret = api_mod.servoJ(dianaJointsPosition, t, ah_t,
                         gain, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def servoL(tcp_pose, t=0.01, ah_t=0.01, gain=300, scale=1000, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = tcp_pose[index]
    active_tcp = c_void_p(0)
    api_mod.servoL.argtypes = [
        POINTER(DIANA_TCP_POSE), c_double, c_double, c_double, c_double, c_void_p, c_char_p]
    api_mod.servoL.restype = c_int
    ret = api_mod.servoL(dianaTcpPose, t, ah_t, gain, scale,
                         active_tcp, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def servoJ_ex(joints_pos, t=0.01, ah_t=0.01, gain=300, realiable=False, ipAddress=''):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    for index in range(0, len(joints_pos)):
        dianaJointsPosition.values[index] = joints_pos[index]
    api_mod.servoJ_ex.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_bool, c_char_p]
    api_mod.servoJ_ex.restype = c_int
    ret = api_mod.servoJ_ex(dianaJointsPosition, t, ah_t,
                            gain, realiable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def servoL_ex(tcp_pose, t=0.01, ah_t=0.01, gain=300, scale=1000, realiable=False, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = tcp_pose[index]
    active_tcp = c_void_p(0)
    api_mod.servoL_ex.argtypes = [POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double, c_double, c_void_p, c_bool, c_char_p]
    api_mod.servoL_ex.restype = c_int
    ret = api_mod.servoL_ex(dianaTcpPose, t, ah_t, gain,
                            scale, active_tcp, realiable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def speedJ_ex(speed, acc, t=0.0, realiable=False, ipAddress=''):
    dianaJointsSpeed = DIANA_JOINTS_SPEED()
    dianaJointsSpeed.values = (c_double * 7)()
    for index in range(0, len(speed)):
        dianaJointsSpeed.values[index] = speed[index]
    api_mod.speedJ_ex.argtypes = [
        POINTER(DIANA_JOINTS_SPEED), c_double, c_double, c_bool, c_char_p]
    api_mod.speedJ_ex.restype = c_int
    ret = api_mod.speedJ_ex(dianaJointsSpeed, acc, t,
                            realiable, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def speedL_ex(speed, acc, t=0.0, realiable=False, ipAddress=''):
    dianaJointsSpeed = DIANA_JOINTS_SPEED_L()
    dianaJointsSpeed.values = (c_double * 6)()
    for index in range(0, len(dianaJointsSpeed.values)):
        dianaJointsSpeed.values[index] = speed[index]
    dianaJointsAcc = DIANA_JOINTS_ACC()
    dianaJointsAcc.values = (c_double * 2)()
    for index in range(0, len(dianaJointsAcc.values)):
        dianaJointsAcc.values[index] = acc[index]
    active_tcp = c_void_p(0)
    api_mod.speedL_ex.argtypes = [POINTER(DIANA_JOINTS_SPEED_L), POINTER(
        DIANA_JOINTS_ACC), c_double, c_void_p, c_bool, c_char_p]
    api_mod.speedL_ex.restype = c_int
    ret = api_mod.speedL_ex(dianaJointsSpeed, dianaJointsAcc,
                            t, active_tcp, realiable, bytes(ipAddress.encode('utf-8')))
    return message(ret)

###################### version 0.9#########################


def dumpToUDisk(ipAddress=''):
    api_mod.dumpToUDisk.argtypes = [c_char_p]
    api_mod.dumpToUDisk.restype = c_int
    ret = api_mod.dumpToUDisk(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def inverse_ext(ref_joints, pose, joints, ipAddress=''):
    refJointsPosition = DIANA_JOINTS_POSITION()
    refJointsPosition.values = (c_double * 7)()
    for index in range(0, len(ref_joints)):
        refJointsPosition.values[index] = ref_joints[index]
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    active_tcp = c_void_p(0)
    api_mod.inverse_ext.argtypes = [POINTER(DIANA_JOINTS_POSITION), POINTER(DIANA_TCP_POSE), POINTER(
        DIANA_JOINTS_POSITION),  c_void_p, c_char_p]
    api_mod.inverse_ext.restype = c_int
    ret = api_mod.inverse_ext(refJointsPosition, dianaTcpPose,
                              byref(dianaJointsPosition), active_tcp, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(joints)):
        joints[index] = dianaJointsPosition.values[index]
    return message(ret)


def getJointLinkPos(joints, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double*7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.getJointLinkPos.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_char_p]
    api_mod.getJointLinkPos.restype = c_int
    ret = api_mod.getJointLinkPos(
        byref(dianaJointPos), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(joints)):
        joints[index] = dianaJointPos.values[index]
    return message(ret)


def createComplexPath(path_type, ipAddress=''):
    complex_path_id = c_uint()
    api_mod.createComplexPath.argtypes = [c_int, POINTER(c_uint), c_char_p]
    api_mod.createComplexPath.restype = c_int
    ret = api_mod.createComplexPath(path_type.value, byref(
        complex_path_id), bytes(ipAddress.encode('utf-8')))
    return ret, complex_path_id


def addMoveLSegmentByTarget(complex_path_id, joints, vel, acc, blendradius, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveLByTarget.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_char_p]
    api_mod.addMoveLByTarget.restype = c_int
    ret = api_mod.addMoveLByTarget(
        complex_path_id, dianaJointPos, vel, acc, blendradius, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addMoveLSegmentByPose(complex_path_id, pose, vel, acc, blendradius, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.addMoveLByPose.argtypes = [c_uint, POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double, c_char_p]
    api_mod.addMoveLByPose.restype = c_int
    ret = api_mod.addMoveLByPose(
        complex_path_id, dianaTcpPose, vel, acc, blendradius, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addMoveJSegmentByTarget(complex_path_id, joints, vel_percent, acc_percent, blendradius_percent, ipAddress=''):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveJByTarget.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_char_p]
    api_mod.addMoveJByTarget.restype = c_int
    ret = api_mod.addMoveJByTarget(
        complex_path_id, dianaJointPos, vel_percent, acc_percent, blendradius_percent, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addMoveJSegmentByPose(complex_path_id, pose, vel_percent, acc_percent, blendradius_percent, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.addMoveJByPose.argtypes = [c_uint, POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double, c_char_p]
    api_mod.addMoveJByPose.restype = c_int
    ret = api_mod.addMoveJByPose(
        complex_path_id, dianaTcpPose, vel_percent, acc_percent, blendradius_percent, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addMoveCSegmentByTarget(complex_path_id, pass_joints, target_joints, vel, acc, blendradius, ignore_rotation, ipAddress=''):
    dianaPassJointPos = DIANA_JOINTS_POSITION()
    dianaPassJointPos.values = (c_double * 7)()
    dianaTargetJointPos = DIANA_JOINTS_POSITION()
    dianaTargetJointPos.values = (c_double * 7)()
    for index in range(0, len(pass_joints)):
        dianaPassJointPos.values[index] = pass_joints[index]
    for index in range(0, len(target_joints)):
        dianaTargetJointPos.values[index] = target_joints[index]
    api_mod.addMoveCByTarget.argtypes = [c_uint, POINTER(DIANA_JOINTS_POSITION), POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_bool, c_char_p]
    api_mod.addMoveCByTarget.restype = c_int
    ret = api_mod.addMoveCByTarget(complex_path_id, dianaPassJointPos,
                                   dianaTargetJointPos, vel, acc, blendradius, ignore_rotation, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addMoveCSegmentByPose(complex_path_id, pass_pose, target_pose, vel, acc, blendradius, ignore_rotation, ipAddress=''):
    dianaPassTcpPose = DIANA_TCP_POSE()
    dianaPassTcpPose.values = (c_double * 6)()
    dianaTargetTcpPose = DIANA_TCP_POSE()
    dianaTargetTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaPassTcpPose.values)):
        dianaPassTcpPose.values[index] = pass_pose[index]
    for index in range(0, len(dianaTargetTcpPose.values)):
        dianaTargetTcpPose.values[index] = target_pose[index]
    api_mod.addMoveCByPose.argtypes = [c_uint, POINTER(DIANA_TCP_POSE), POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double, c_bool, c_char_p]
    api_mod.addMoveCByPose.restype = c_int
    ret = api_mod.addMoveCByPose(complex_path_id, dianaPassTcpPose,
                                 dianaTargetTcpPose, vel, acc, blendradius, ignore_rotation, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def runComplexPath(complex_path_id, ipAddress=''):
    api_mod.runComplexPath.argtypes = [c_uint, c_char_p]
    api_mod.runComplexPath.restype = c_int
    ret = api_mod.runComplexPath(
        complex_path_id, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def destroyComplexPath(complex_path_id, ipAddress=''):
    api_mod.destroyComplexPath.argtypes = [c_uint, c_char_p]
    api_mod.destroyComplexPath.restype = c_int
    ret = api_mod.destroyComplexPath(complex_path_id, bytes(
        ipAddress.encode('utf-8')), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def saveEnvironment(ipAddress=''):
    api_mod.saveEnvironment.argtypes = [c_char_p]
    api_mod.saveEnvironment.restype = c_int
    ret = api_mod.saveEnvironment(bytes(ipAddress.encode('utf-8')))
    return message(ret)

###################### version 0.10#########################


def dumpToUDiskEx(timeout_second, ipAddress=''):
    api_mod.dumpToUDiskEx.restype = c_int
    api_mod.dumpToUDiskEx.argtypes = [c_double, c_char_p]
    ret = api_mod.dumpToUDiskEx(
        timeout_second, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def enterForceMode_ex(forceDirection, forceValue, maxApproachVelocity, maxAllowTcpOffset, active_tcp=None, ipAddress=''):
    arr_ForceDirection = DIANA_FORCE_DIRECTION()
    arr_ForceDirection.values = (c_double*3)()
    for index in range(0, len(arr_ForceDirection.values)):
        arr_ForceDirection.values[index] = forceDirection[index]
    ret = 0
    if active_tcp==None:
        use_active_tcp = c_void_p(0)
        api_mod.enterForceMode_ex.restype = c_int
        api_mod.enterForceMode_ex.argtypes = [POINTER(
            DIANA_FORCE_DIRECTION), c_double, c_double, c_double, c_void_p, c_char_p]
        ret = api_mod.enterForceMode_ex(byref(
            arr_ForceDirection), forceValue, maxApproachVelocity, maxAllowTcpOffset, use_active_tcp, bytes(ipAddress.encode('utf-8')))
    else:
        arr_Active_Tcp = DIANA_ACTIVE_TCP()
        arr_Active_Tcp.values = (c_double*6)()
        for index in range(0, len(arr_Active_Tcp.values)):
            arr_Active_Tcp.values[index] = active_tcp[index]
        api_mod.enterForceMode_ex.restype = c_int
        api_mod.enterForceMode_ex.argtypes = [POINTER(
            DIANA_FORCE_DIRECTION), c_double, c_double, c_double, POINTER(DIANA_ACTIVE_TCP), c_char_p]
        ret = api_mod.enterForceMode_ex(byref(
            arr_ForceDirection), forceValue, maxApproachVelocity, maxAllowTcpOffset, byref(arr_Active_Tcp), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def readDI(group_name, di_name, ipAddress=''):
    DIValue = c_int()
    api_mod.readDI.argtypes = [c_char_p, c_char_p, POINTER(c_int), c_char_p]
    api_mod.readDI.restype = c_int
    ret = api_mod.readDI(bytes(group_name, encoding="utf-8"),
                         bytes(di_name, encoding="utf-8"), byref(DIValue), bytes(ipAddress.encode('utf-8')))
    return ret, DIValue


def readAI(group_name, di_name, ipAddress=''):
    AIValue = c_double()
    AIMode = c_int()
    api_mod.readAI.argtypes = [c_char_p, c_char_p,
                               POINTER(c_int), POINTER(c_double), c_char_p]
    api_mod.readAI.restype = c_int
    ret = api_mod.readAI(bytes(group_name, encoding="utf-8"),
                         bytes(di_name, encoding="utf-8"), byref(AIMode), byref(AIValue), bytes(ipAddress.encode('utf-8')))
    return ret, AIMode, AIValue


def setAIMode(group_name, di_name, mode, ipAddress=''):
    api_mod.setAIMode.argtypes = [c_char_p, c_char_p, c_int, c_char_p]
    api_mod.setAIMode.restype = c_int
    ret = api_mod.setAIMode(
        bytes(group_name, encoding="utf-8"), bytes(di_name, encoding="utf-8"), mode, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def writeDO(group_name, di_name, value, ipAddress=''):
    api_mod.writeDO.argtypes = [c_char_p, c_char_p, c_int, c_char_p]
    api_mod.writeDO.restype = c_int
    ret = api_mod.writeDO(bytes(group_name, encoding="utf-8"),
                          bytes(di_name, encoding="utf-8"), value, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def writeAO(group_name, di_name, mode, value, ipAddress=''):
    api_mod.writeAO.argtypes = [c_char_p, c_char_p, c_int, c_double, c_char_p]
    api_mod.writeAO.restype = c_int
    ret = api_mod.writeAO(bytes(group_name, encoding="utf-8"),
                          bytes(di_name, encoding="utf-8"), mode, value, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def readBusCurrent(ipAddress=''):
    Current = c_double()
    api_mod.readBusCurrent.argtypes = [POINTER(c_double), c_char_p]
    api_mod.readBusCurrent.restype = c_int
    ret = api_mod.readBusCurrent(byref(Current), bytes(ipAddress.encode('utf-8')))
    return ret, Current


def readBusVoltage(ipAddress=''):
    Voltage = c_double()
    api_mod.readBusVoltage.argtypes = [POINTER(c_double), c_char_p]
    api_mod.readBusVoltage.restype = c_int
    ret = api_mod.readBusVoltage(byref(Voltage), bytes(ipAddress.encode('utf-8')))
    return ret, Voltage

###################### version 1.10#########################


def getDH(aDH, alphaDH, dDH, thetaDH, ipAddress=''):
    arr_aDH = DIANA_DH_A()
    arr_aDH.values = (c_double*7)()
    arr_alphaDH = DIANA_DH_Alpha()
    arr_alphaDH.values = (c_double*7)()
    arr_dDH = DIANA_DH_D()
    arr_dDH.values = (c_double*7)()
    arr_thetaDH = DIANA_DH_Theta()
    arr_thetaDH.values = (c_double*7)()
    api_mod.getDH.restype = c_int
    api_mod.getDH.argtypes = [POINTER(DIANA_DH_A),
                              POINTER(DIANA_DH_Alpha), POINTER(DIANA_DH_D), POINTER(DIANA_DH_Theta), c_char_p]
    ret = api_mod.getDH(byref(arr_aDH), byref(arr_alphaDH),
                        byref(arr_dDH), byref(arr_thetaDH), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(aDH)):
        aDH[index] = arr_aDH.values[index]
    for index in range(0, len(alphaDH)):
        alphaDH[index] = arr_alphaDH.values[index]
    for index in range(0, len(dDH)):
        dDH[index] = arr_dDH.values[index]
    for index in range(0, len(thetaDH)):
        thetaDH[index] = arr_thetaDH.values[index]
    return message(ret)


def getOriginalJointTorque(torques, ipAddress=''):
    jointTorque = DIANA_JOINT_TORQUE()
    jointTorque.values = (c_double*7)()
    api_mod.getOriginalJointTorque.restype = c_int
    api_mod.getOriginalJointTorque.argtypes = [
        POINTER(DIANA_JOINT_TORQUE), c_char_p]
    ret = api_mod.getOriginalJointTorque(
        byref(jointTorque), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(torques)):
        torques[index] = jointTorque.values[index]
    return message(ret)


def getJacobiMatrix(matrix_jacobi, ipAddress=''):
    jacobi_matrix = DIANA_JACOBI_MATRIX()
    jacobi_matrix.values = (c_double*42)()
    api_mod.getJacobiMatrix.restype = c_int
    api_mod.getJacobiMatrix.argtypes = [POINTER(DIANA_JACOBI_MATRIX), c_char_p]
    ret = api_mod.getJacobiMatrix(
        byref(jacobi_matrix), bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(matrix_jacobi)):
        matrix_jacobi[index] = jacobi_matrix.values[index]
    return message(ret)

###################### version 1.20#########################


def resetDH(ipAddress=''):
    api_mod.resetDH.argtypes = [c_char_p]
    api_mod.resetDH.restype = c_int
    ret = api_mod.resetDH(bytes(ipAddress.encode('utf-8')))
    return message(ret)

###################### version 1.30#########################


def runProgram(name, ipAddress=''):
    api_mod.runProgram.argtypes = [c_char_p, c_char_p]
    api_mod.runProgram.restype = c_int
    ret = api_mod.runProgram(
        bytes(name, encoding="utf-8"), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def stopProgram(name, ipAddress=''):
    api_mod.stopProgram.argtypes = [c_char_p, c_char_p]
    api_mod.stopProgram.restype = c_int
    ret = api_mod.stopProgram(
        bytes(name, encoding="utf-8"), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def getVariableValue(name, ipAddress=''):
    api_mod.getVariableValue.argtypes = [c_char_p, POINTER(c_double), c_char_p]
    api_mod.getVariableValue.restype = c_int
    value = c_double()
    ret = api_mod.getVariableValue(
        bytes(name, encoding="utf-8"), byref(value), bytes(ipAddress.encode('utf-8')))
    return message(ret), value.value


def setVariableValue(name, value, ipAddress=''):
    val = c_double(value)
    api_mod.setVariableValue.argtypes = [c_char_p, POINTER(c_double), c_char_p]
    api_mod.setVariableValue.restype = c_int
    ret = api_mod.setVariableValue(
        bytes(name, encoding="utf-8"), byref(val), bytes(ipAddress.encode('utf-8')))
    return message(ret)


def isTaskRunning(name, ipAddress=''):
    api_mod.isTaskRunning.argtypes = [c_char_p, c_char_p]
    api_mod.isTaskRunning.restype = c_int
    ret = api_mod.isTaskRunning(
        bytes(name, encoding="utf-8"), bytes(ipAddress.encode('utf-8')))
    if(ret >= 0):
        return True
    if(ret== -1):
        return False
    else:
        errorcode = getLastError()
        errorMessage = errorCodeMessage.get(errorcode)
        callerName = 'function ' + sys._getframe().f_back.f_code.co_name + ' fails,'
        if errorMessage == None:
            print(callerName + 'Error Code ' + str(errorcode))
        else:
            print(callerName + errorMessage)
        return False


def pauseProgram(ipAddress=''):
    api_mod.stopProgram.argtypes = [c_char_p]
    api_mod.pauseProgram.restype = c_int
    ret = api_mod.pauseProgram(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def resumeProgram(ipAddress=''):
    api_mod.resumeProgram.argtypes = [c_char_p]
    api_mod.resumeProgram.restype = c_int
    ret = api_mod.resumeProgram(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def stopAllProgram(ipAddress=''):
    api_mod.stopAllProgram.argtypes = [c_char_p]
    api_mod.stopAllProgram.restype = c_int
    ret = api_mod.stopAllProgram(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def isAnyTaskRunning(ipAddress=''):
    api_mod.isAnyTaskRunning.argtypes = [c_char_p]
    api_mod.isAnyTaskRunning.restype = c_int
    ret = api_mod.isAnyTaskRunning(bytes(ipAddress.encode('utf-8')))
    if(ret >= 0):
        return True
    if(ret== -1):
        return False
    else:
        errorcode = getLastError()
        errorMessage = errorCodeMessage.get(errorcode)
        callerName = 'function ' + sys._getframe().f_back.f_code.co_name + ' fails,'
        if errorMessage == None:
            print(callerName + 'Error Code ' + str(errorcode))
        else:
            print(callerName + errorMessage)
        return False

###################### version 1.40#########################


def cleanErrorInfo(ipAddress=''):
    api_mod.cleanErrorInfo.argtypes = [c_char_p]
    api_mod.cleanErrorInfo.restype = c_int
    ret = api_mod.cleanErrorInfo(bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setCollisionLevel(level, ipAddress=''):
    if not isinstance(level, collision_level):
        raise TypeError('level must be an instance of collision_level Enum')
    api_mod.setCollisionLevel.argtypes = [c_int, c_char_p]
    api_mod.setCollisionLevel.restype = c_int
    ret = api_mod.setCollisionLevel(
        level.value, bytes(ipAddress.encode('utf-8')))
    return message(ret)


###################### version 1.50#########################

def getJointCount(ipAddress=''):
    api_mod.getJointCount.argtypes = [c_char_p]
    api_mod.getJointCount.restype = c_int
    count = api_mod.getJointCount(bytes(ipAddress.encode('utf-8')))
    return count


def getWayPoint(waypointName, dblTcppos, dblJoints, ipAddress=''):
    api_mod.getWayPoint.restype = c_int
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    api_mod.getWayPoint.argtypes = [
        c_char_p, POINTER(DIANA_TCP_POSE), POINTER(DIANA_JOINTS_POSITION), c_char_p]
    ret = api_mod.getWayPoint(
        bytes(waypointName.encode('utf-8')), dianaTcpPose, dianaJointPos, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(dblTcppos)):
        dblTcppos[index] = dianaTcpPose.values[index]
    for index in range(0, len(dblJoints)):
        dblJoints[index] = dianaJointPos.values[index]
    return message(ret)

def setWayPoint(waypointName, dblTcppos, dblJoints, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dblTcppos)):
        dianaTcpPose.values[index] = dblTcppos[index]
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(dblJoints)):
        dianaJointPos.values[index] = dblJoints[index]
    api_mod.setWayPoint.restype = c_int
    api_mod.setWayPoint.argtypes = [
        c_char_p, POINTER(DIANA_TCP_POSE), POINTER(DIANA_JOINTS_POSITION), c_char_p]
    ret = api_mod.setWayPoint(
        bytes(waypointName.encode('utf-8')), dianaTcpPose, dianaJointPos, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def addWayPoint(waypointName, dblTcppos, dblJoints, ipAddress=''):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dblTcppos)):
        dianaTcpPose.values[index] = dblTcppos[index]
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(dblJoints)):
        dianaJointPos.values[index] = dblJoints[index]
    api_mod.addWayPoint.restype = c_int
    api_mod.addWayPoint.argtypes = [
        c_char_p, POINTER(DIANA_TCP_POSE), POINTER(DIANA_JOINTS_POSITION), c_char_p]
    ret = api_mod.addWayPoint(
        bytes(waypointName.encode('utf-8')), dianaTcpPose, dianaJointPos, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def deleteWayPoint(waypointName, ipAddress=''):
    api_mod.deleteWayPoint.argtypes = [c_char_p, c_char_p]
    api_mod.deleteWayPoint.restype = c_int
    ret = api_mod.deleteWayPoint(
        bytes(waypointName.encode('utf-8')), bytes(ipAddress.encode('utf-8')))
    return message(ret)


###################### version 1.60#########################


def getDefaultActiveTcp(default_tcp, ipAddress=''):
    defaultTcp = DIANA_DEFAULT_TCP()
    defaultTcp.values = (c_double*16)()
    api_mod.getDefaultActiveTcp.argtypes = [
        POINTER(DIANA_DEFAULT_TCP), c_char_p]
    api_mod.getDefaultActiveTcp.restype = c_int
    ret = api_mod.getDefaultActiveTcp(
        defaultTcp, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(defaultTcp.values)):
        default_tcp[index] = defaultTcp.values[index]
    return message(ret)


def getDefaultActiveTcpPose(arrPose, ipAddress=''):
    pose = DIANA_TCP_POSE()
    pose.values = (c_double*6)()
    api_mod.getDefaultActiveTcpPose.argtypes = [
        POINTER(DIANA_TCP_POSE), c_char_p]
    api_mod.getDefaultActiveTcpPose.restype = c_int
    ret = api_mod.getDefaultActiveTcpPose(
        pose, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(pose.values)):
        arrPose[index] = pose.values[index]
    return message(ret)


def getActiveTcpPayload(payload, ipAddress=''):
    tcpPayload = DIANA_TCP_PAYLOAD()
    tcpPayload.values = (c_double*10)()
    api_mod.getActiveTcpPayload.argtypes = [
        POINTER(DIANA_TCP_PAYLOAD), c_char_p]
    api_mod.getActiveTcpPayload.restype = c_int
    ret = api_mod.getActiveTcpPayload(
        tcpPayload, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(tcpPayload.values)):
        payload[index] = tcpPayload.values[index]
    return message(ret)


def zeroSpaceManualMove(direction, dblJointsVel, dblJointAcc, ipAddress=''):
    if not isinstance(direction, zero_space_move_direction):
        raise TypeError(
            'direction must be an instance of zero_space_move_direction Enum')
    joint_vel = DIANA_JOINT_ANGULAR_VEL()
    joint_vel.values = (c_double*7)()
    for index in range(0, len(dblJointsVel)):
        joint_vel.values[index] = dblJointsVel[index]
    joint_acc = DIANA_JOINT_ACC()
    joint_acc.values = (c_double*7)()
    for index in range(0, len(joint_acc.values)):
        joint_acc.values[index] = dblJointAcc[index]
    api_mod.zeroSpaceManualMove.argtypes = [c_int, POINTER(
        DIANA_JOINT_ANGULAR_VEL), POINTER(DIANA_JOINT_ACC), c_char_p]
    api_mod.zeroSpaceManualMove.restype = c_int
    ret = api_mod.zeroSpaceManualMove(
        direction.value, joint_vel, joint_acc, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def moveTcp_ex(coordinate, direction, velocity, accleartion, ipAddress=''):
    if not isinstance(coordinate, coordinate_e):
        raise TypeError(
            'coordinate must be an instance of coordinate_e Enum')
    if not isinstance(direction, tcp_direction_e):
        raise TypeError(
            'direction must be an instance of tcp_direction_e Enum')
    api_mod.moveTcp_ex.argtypes = [c_int, c_int, c_double, c_double, c_char_p]
    api_mod.moveTcp_ex.restype = c_int
    ret = api_mod.moveTcp_ex(
        coordinate.value, direction.value, velocity, accleartion, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def setExternalAppendTorCutoffFreq(dblFreq, ipAddress=''):
    api_mod.setExternalAppendTorCutoffFreq.argtypes = [c_double, c_char_p]
    api_mod.setExternalAppendTorCutoffFreq.restype = c_int
    ret = api_mod.setExternalAppendTorCutoffFreq(
        dblFreq, bytes(ipAddress.encode('utf-8')))
    return message(ret)


def poseTransform(srcPose, srcMatrixPose, dstMartixPose, dstPose):
  srcTcpPose = DIANA_TCP_POSE()
  srcTcpPose.values = (c_double * 6)()
  for index in range(0, len(srcTcpPose.values)):
    srcTcpPose.values[index] = srcPose[index]
  srcMatrix = DIANA_TCP_POSE()
  srcMatrix.values = (c_double * 6)()
  for index in range(0, len(srcMatrix.values)):
    srcMatrix.values[index] = srcMatrixPose[index]
  dstTcpPose = DIANA_TCP_POSE()
  dstTcpPose.values = (c_double * 6)()
  for index in range(0, len(dstTcpPose.values)):
    dstTcpPose.values[index] = dstMartixPose[index]
  dstTarPose = DIANA_TCP_POSE()
  dstTarPose.values = (c_double * 6)()
  for index in range(0, len(dstTarPose.values)):
    dstPose[index] = -1
  api_mod.poseTransform.argtypes = [POINTER(DIANA_TCP_POSE), POINTER(
      DIANA_TCP_POSE), POINTER(DIANA_TCP_POSE), POINTER(DIANA_TCP_POSE)]
  api_mod.poseTransform.restype = c_int
  ret = api_mod.poseTransform(byref(srcTcpPose), byref(
      srcMatrix), byref(dstTcpPose), byref(dstTarPose))
  for index in range(0, len(dstTarPose.values)):
      dstPose[index] = dstTarPose.values[index]
  return message(ret)

def updateForce(forceDirection,forceValue, ipAddress=''):
    arr_ForceDirection = DIANA_FORCE_DIRECTION()
    arr_ForceDirection.values = (c_double*3)()
    for index in range(0, len(arr_ForceDirection.values)):
        arr_ForceDirection.values[index] = forceDirection[index]
    api_mod.updateForce.restype = c_int
    api_mod.updateForce.argtypes = [POINTER(DIANA_FORCE_DIRECTION),c_double, c_char_p]
    ret = api_mod.updateForce(byref(arr_ForceDirection),forceValue, bytes(ipAddress.encode('utf-8')))
    return message(ret)

def updateForce_ex(forceDirection,forceValue, active_tcp=None, ipAddress=''):
    arr_ForceDirection = DIANA_FORCE_DIRECTION()
    arr_ForceDirection.values = (c_double*3)()
    for index in range(0, len(arr_ForceDirection.values)):
        arr_ForceDirection.values[index] = forceDirection[index]
    ret = 0
    if active_tcp==None:
        use_active_tcp = c_void_p(0)
        api_mod.updateForce_ex.restype = c_int
        api_mod.updateForce_ex.argtypes = [POINTER(DIANA_FORCE_DIRECTION),c_double,c_void_p, c_char_p]
        ret = api_mod.updateForce_ex(byref(arr_ForceDirection),forceValue,use_active_tcp, bytes(ipAddress.encode('utf-8')))
    else:
        arr_Active_Tcp = DIANA_ACTIVE_TCP()
        arr_Active_Tcp.values = (c_double*6)()
        for index in range(0, len(arr_Active_Tcp.values)):
            arr_Active_Tcp.values[index] = active_tcp[index]
        api_mod.updateForce_ex.restype = c_int
        api_mod.updateForce_ex.argtypes = [POINTER(DIANA_FORCE_DIRECTION),c_double,POINTER(DIANA_ACTIVE_TCP), c_char_p]
        ret = api_mod.updateForce_ex(byref(arr_ForceDirection),forceValue,byref(arr_Active_Tcp), bytes(ipAddress.encode('utf-8')))
    return message(ret)

def inverseClosedFull(pose, lock_joint_index, lock_joint_position, ref_joints, active_tcp=None, ipAddress=''):
    refJointsPosition = DIANA_JOINTS_POSITION()
    refJointsPosition.values = (c_double * 7)()
    for index in range(0, len(ref_joints)):
        refJointsPosition.values[index] = ref_joints[index]
    dianaPose = DIANA_TCP_POSE()
    dianaPose.values = (c_double * 6)()
    for index in range(0, len(pose)):
        dianaPose.values[index] = pose[index]
    ret = 0
    if active_tcp==None:
        use_active_tcp = c_void_p(0)
        api_mod.inverseClosedFull.argtypes = [POINTER(DIANA_TCP_POSE),c_int,c_double,POINTER(DIANA_JOINTS_POSITION),c_void_p, c_char_p]
        api_mod.inverseClosedFull.restype = c_int
        id = api_mod.inverseClosedFull(byref(dianaPose),lock_joint_index,lock_joint_position,byref(refJointsPosition), use_active_tcp, bytes(ipAddress.encode('utf-8')))
    else:
        arrActiveTcp = DIANA_ACTIVE_TCP()
        arrActiveTcp.values = (c_double*6)()
        for index in range(0, len(arrActiveTcp.values)):
            arrActiveTcp.values[index] = active_tcp[index]
        api_mod.inverseClosedFull.argtypes = [POINTER(DIANA_TCP_POSE),c_int,c_double,POINTER(DIANA_JOINTS_POSITION),POINTER(DIANA_ACTIVE_TCP), c_char_p]
        api_mod.inverseClosedFull.restype = c_int
        id = api_mod.inverseClosedFull(byref(dianaPose),lock_joint_index,lock_joint_position,byref(refJointsPosition), byref(arrActiveTcp), bytes(ipAddress.encode('utf-8')))
    return id

def getInverseClosedResultSize(id, ipAddress=''):
    api_mod.getInverseClosedResultSize.argtypes = [c_int, c_char_p]
    api_mod.getInverseClosedResultSize.restype = c_int
    size = api_mod.getInverseClosedResultSize(id, bytes(ipAddress.encode('utf-8')))
    return size

def getInverseClosedJoints(id, index, joints, ipAddress=''):
    dianaJoints = DIANA_JOINTS_POSITION()
    dianaJoints.values = (c_double * 7)()
    api_mod.getInverseClosedJoints.argtypes = [c_int, c_int, POINTER(DIANA_JOINTS_POSITION), c_char_p]
    api_mod.getInverseClosedJoints.restype = c_int
    size = api_mod.getInverseClosedJoints(id, index, dianaJoints, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(joints)):
        joints[index] = dianaJoints.values[index]
    return size

def destoryInverseClosedItems(id, ipAddress=''):
    api_mod.destoryInverseClosedItems.argtypes = [c_int, c_char_p]
    api_mod.destoryInverseClosedItems.restype = c_int
    ret = api_mod.destoryInverseClosedItems(id, bytes(ipAddress.encode('utf-8')))
    return message(ret)

def getGravInfo(grav, ipAddress=''):
    gravInfo = DIANA_GRAV_INFO()
    gravInfo.values = (c_double * 3)()
    api_mod.getGravInfo.argtypes = [POINTER(DIANA_GRAV_INFO), c_char_p]
    api_mod.getGravInfo.restype = c_int
    ret = api_mod.getGravInfo(gravInfo, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(grav)):
        grav[index] = gravInfo.values[index]
    return message(ret)

def setGravInfo(grav, ipAddress=''):
    gravInfo = DIANA_GRAV_INFO()
    gravInfo.values = (c_double * 3)()
    for index in range(0, len(grav)):
        gravInfo.values[index] = grav[index]
    api_mod.setGravInfo.argtypes = [POINTER(DIANA_GRAV_INFO), c_char_p]
    api_mod.setGravInfo.restype = c_int
    ret = api_mod.setGravInfo(gravInfo, bytes(ipAddress.encode('utf-8')))
    return message(ret)

def getGravAxis(grav, ipAddress=''):
    gravAxis = DIANA_AXIS_VECTOR()
    gravAxis.values = (c_double * 3)()
    api_mod.getGravAxis.argtypes = [POINTER(DIANA_AXIS_VECTOR), c_char_p]
    api_mod.getGravAxis.restype = c_int
    ret = api_mod.getGravAxis(gravAxis, bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(grav)):
        grav[index] = gravAxis.values[index]
    return message(ret)

def setGravAxis(grav, ipAddress=''):
    gravInfo = DIANA_AXIS_VECTOR()
    gravInfo.values = (c_double * 3)()
    for index in range(0, len(grav)):
        gravInfo.values[index] = grav[index]
    api_mod.setGravAxis.argtypes = [POINTER(DIANA_AXIS_VECTOR), c_char_p]
    api_mod.setGravAxis.restype = c_int
    ret = api_mod.setGravAxis(gravInfo, bytes(ipAddress.encode('utf-8')))
    return message(ret)

def speedLOnTcp(speed, acc, t=0.0, ipAddress=''):
    dianaJointsSpeed = DIANA_JOINTS_SPEED_L()
    dianaJointsSpeed.values = (c_double * 6)()
    for index in range(0, len(dianaJointsSpeed.values)):
        dianaJointsSpeed.values[index] = speed[index]
    dianaJointsAcc = DIANA_JOINTS_ACC()
    dianaJointsAcc.values = (c_double * 2)()
    for index in range(0, len(dianaJointsAcc.values)):
        dianaJointsAcc.values[index] = acc[index]
    active_tcp = c_void_p(0)
    api_mod.speedLOnTcp.argtypes = [POINTER(DIANA_JOINTS_SPEED_L), POINTER(
        DIANA_JOINTS_ACC), c_double, c_void_p, c_char_p]
    api_mod.speedLOnTcp.restype = c_int
    ret = api_mod.speedLOnTcp(dianaJointsSpeed, dianaJointsAcc,
                            t, active_tcp, bytes(ipAddress.encode('utf-8')))
    return message(ret)
    
def getTcpForceInToolCoordinate(forces, ipAddress=''):
    dianaTcpForce = DIANA_TCP_FORCE()
    dianaTcpForce.values = (c_double * 6)()
    api_mod.getTcpForceInToolCoordinate.argtypes = [POINTER(DIANA_TCP_FORCE), c_char_p]
    api_mod.getTcpForceInToolCoordinate.restype = c_int
    ret = api_mod.getTcpForceInToolCoordinate(byref(dianaTcpForce),
                              bytes(ipAddress.encode('utf-8')))
    for index in range(0, len(dianaTcpForce.values)):
        forces[index] = dianaTcpForce.values[index]
    return message(ret)

def calculateJacobi(matrixJacobi, joints, ipAddress=''):
    _joint_count = getJointCount(ipAddress)
    dianaJoints = DIANA_JOINTS_POSITION()
    dianaJoints.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJoints.values[index] = joints[index]
    dianaJacobiMatrix = DIANA_JACOBI_MATRIX()
    dianaJacobiMatrix.values = (c_double * 42)()
    api_mod.calculateJacobi.argtypes = [POINTER(DIANA_JACOBI_MATRIX), POINTER(DIANA_JOINTS_POSITION), c_int, c_char_p]
    api_mod.calculateJacobi.restype = c_int
    ret = api_mod.calculateJacobi(byref(dianaJacobiMatrix), dianaJoints, _joint_count, bytes(ipAddress.encode('utf-8')))
    if ret < 0:
        return message(ret)
    for index in range(0, _joint_count * 6):
        matrixJacobi[index] = dianaJacobiMatrix.values[index]
    return message(ret)
                             
def calculateJacobiTF(matrixJacobi, joints, toolMatrix, ipAddress=''):
    _joint_count = getJointCount(ipAddress)
    dianaJoints = DIANA_JOINTS_POSITION()
    dianaJoints.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJoints.values[index] = joints[index]
    dianaToolMatrix = DIANA_ACTIVE_TCP()
    dianaToolMatrix.values = (c_double * 6)()
    for index in range(0, len(toolMatrix)):
        dianaToolMatrix.values[index] = toolMatrix[index]
    dianaJacobiMatrix = DIANA_JACOBI_MATRIX()
    dianaJacobiMatrix.values = (c_double * 42)()
    api_mod.calculateJacobiTF.argtypes = [POINTER(DIANA_JACOBI_MATRIX),POINTER(DIANA_JOINTS_POSITION), c_int, POINTER(DIANA_ACTIVE_TCP), c_char_p]
    api_mod.calculateJacobiTF.restype = c_int
    ret = api_mod.calculateJacobiTF(byref(dianaJacobiMatrix), dianaJoints, _joint_count, dianaToolMatrix, bytes(ipAddress.encode('utf-8')))
    if ret < 0 :
        return message(ret)
    for index in range(0, _joint_count * 6):
        matrixJacobi[index] = dianaJacobiMatrix.values[index]
    return message(ret)
