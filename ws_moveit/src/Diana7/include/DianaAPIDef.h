#pragma once

#pragma pack(1)

#include <stdint.h>
#ifndef _WIN32
#include <sys/types.h>
#endif

#define _API_SUPPORT_V2_

#define MAX_SUPPORT_ROBOTARM_NUM 3

extern "C" typedef void (*FNCERRORCALLBACK)(int e);

// TrajectoryState
extern "C" typedef struct _TrajectoryState {
    int taskId;
    int segCount;
    int segIndex;
    int errorCode;
    int isPaused;
    int isFreeDriving;
    int isZeroSpaceFreeDriving;
} StrTrajectoryState;

extern "C" typedef struct _ErrorInfo {
    int errorId;
    int errorType;
    int errorCode;
    char errorMsg[64];
} StrErrorInfo;
//用户层机械臂反馈状态结构体
extern "C" typedef struct _RobotStateInfo {
    double jointPos[7];
    double jointAngularVel[7];
    double jointCurrent[7];
    double jointTorque[7];
    double tcpPos[6];
    double tcpExternalForce;
    bool bCollision;
    bool bTcpForceValid;
    double tcpForce[6];
    double jointForce[7];
    StrTrajectoryState trajState;
    StrErrorInfo errorInfo;
} StrRobotStateInfo;

#define USER_MAXIMUM_DOUBLE_SIZE 40
#define USER_MAXIMUM_INT8_SIZE 160

extern "C" typedef struct _CustomStateInfo {
    double dblField[USER_MAXIMUM_DOUBLE_SIZE];
    int8_t int8Field[USER_MAXIMUM_INT8_SIZE];
} StrCustomStateInfo;

extern "C" typedef void (*FNCSTATECALLBACK)(StrRobotStateInfo *pinfo);

#pragma pack()