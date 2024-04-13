import time
import sys
sys.path.append('/home/ws_moveit/src/Diana7/bin')
import DianaApi as D
from jodellSdk.jodellSdkDemo import ClawEpgTool

class Actuator(object):
    def __init__(self, debug=False):
        self.ip_addr = '192.168.10.75'
        self.debug = debug
        def errorCallback(e):
            print("error code" + str(e))
        def robotStateCallback(stateInfo):
            for i in range(0,7):
                print(stateInfo.contents.jointPos(i))
            for i in range(0,7):
                print(stateInfo.contents. jointAngularVel(i))
        self.fnError = D.FNCERRORCALLBACK(errorCallback)
        self.fnState = D.FNCSTATECALLBACK(robotStateCallback)
        self.netInfo=('192.168.10.75', 0, 0, 0, 0, 0)
        D.initSrv(self.netInfo, self.fnError, None)
        # claw_control
        self.clawTool = ClawEpgTool()
        self.clawId = 9
        self.bandRate = 115200
        self.comList = self.clawTool.searchCom()
        self.clawTool.serialOperation(self.comList[1], self.bandRate, True)
        self.clawTool.clawEnable(self.clawId, True)

    def zero(self, vel=0.2, acc=0.1):
        joints = (0.0,0.0,0.0,0.0,0.0,0.0,0.0)
        print("Returning to zero....")
        D.moveJToTarget(joints, vel, acc, self.ip_addr)
        D.wait_move()
        print("Return to zero successfully!")

    def run_path_between_points(self, points, vel=0.1, acc=0.5):
        counter = 0
        print("Running path between points....")
        for point in points:
            if self.debug:
                print("Move from point {} to {}".format(counter, counter+1))
            D.moveJToTarget(point, vel, acc, self.ip_addr)
            D.wait_move()
            counter = counter + 1
        print("Arrive the destination")

    def run_complex_path(self, points, vel=0.05, acc=0.05):
        ret = D.createComplexPath(D.complex_path_type.NORMAL_JOINT_PATH, self.ip_addr)
        if ret[0]==0:
            for point in points:
                D.addMoveJSegmentByTarget(ret[1],point, vel, acc, 0.1, self.ip_addr)
            print("Running complex path....")
            D.runComplexPath(ret[1], self.ip_addr)
            D.wait_move()
            print("Run complex path successfully!")
        D.destroyComplexPath(ret[1])

    def pick_up(self, vel=0.03, acc=0.05):
        param_z = D.tcp_direction_e.T_MOVE_Z_UP
        param_x = D.tcp_direction_e.T_MOVE_X_DOWN
        param_y = D.tcp_direction_e.T_MOVE_Y_DOWN
        D.moveTCP(param_z, vel, acc, self.ip_addr)
        time.sleep(4)
        D.moveTCP(param_x, vel, acc, self.ip_addr)
        time.sleep(4)

    def pull_place(self, vel=0.03, acc=0.05):
        param_z_u = D.tcp_direction_e.T_MOVE_Z_UP
        param_z_d = D.tcp_direction_e.T_MOVE_Z_DOWN
        param_y = D.tcp_direction_e.T_MOVE_Y_DOWN
        param_x_d = D.tcp_direction_e.T_MOVE_X_DOWN
        D.moveTCP(param_z_u, vel, acc, self.ip_addr)
        time.sleep(3)
        D.moveTCP(param_y, vel, acc, self.ip_addr)
        time.sleep(6)
        D.moveTCP(param_z_d, vel, acc, self.ip_addr)
        time.sleep(2.5)
        self.claw_open()
        D.moveTCP(param_x_d, vel, acc, self.ip_addr)
        time.sleep(4)

    def claw_open(self):
        self.clawTool.runWithoutParam(self.clawId, 0x1)
        self.claw_waitmove()

    def claw_close(self, pos=0xFF, vel=0x10, f=0x20):
        self.clawTool.runWithParam(self.clawId, pos, vel, f)
        self.claw_waitmove()
    
    def close_actuator(self):
        D.destroySrv(self.ip_addr)
        self.clawTool.clawEnable(self.clawId, False)
        self.clawTool.serialOperation(self.comList[0], self.bandRate, False)

    def claw_waitmove(self):
        time.sleep(0.5)
        while True:
            speed = self.clawTool.getClawCurrentSpeed(self.clawId)
            if speed[0]!=0:
                time.sleep(0.01)
            else:
                break
    


if __name__ == "__main__":
    actuator = Actuator()

    
    try:
        # time.sleep(1)
        # actuator.claw_open()
        # time.sleep(10)
        # actuator.claw_close()
        actuator.zero(vel=0.2, acc=0.1)
        # actuator.pull_place()
        actuator.claw_open()

    finally:
        actuator.close_actuator()