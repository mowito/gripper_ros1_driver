#!/usr/bin/env python3

from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippersRobotInput as inputMsg
from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippersRobotOutput as outputMsg
from robotiq_vacuum_grippers_control.srv import GripperCmd,GripperCmdResponse
 
import std_srvs
from std_srvs.srv import SetBool,SetBoolResponse
from std_srvs.srv import Empty,EmptyResponse
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int16


class RobotiqVGripper(Node):
    def __init__(self):
        super().__init__('RobotiqVGripper')
        self.cur_status = None
        self.status_sub =  self.create_subscription( inputMsg, 'RobotiqVacuumGrippersRobotInput', self._status_cb)
        assert self.status_sub
        self.cmd_pub =  self.create_publisher(outputMsg, 'RobotiqVacuumGrippersRobotOutput')
        self.release_try_limit = 10
        self.timeoutSmall = 0.05
        self.timeoutLarge = 1.5
        self.timeoutminimum=0.005
        self.on_count_pub =  self.create_publisher(Int16, '/on_service_count')
        self.off_count_pub = self.create_publisher(Int16, '/off_service_count')
        self.on_count = 0
        self.off_count = 0
        serviceOn =  self.create_service(std_srvs.srv.SetBool, '/on', self.callbackOn) 
        serviceOff =  self.create_service(std_srvs.srv.SetBool, '/off', self.callbackOff)    

        serviceObjectDetected = self.create_service(std_srvs.srv.SetBool, '/grip_status', self.callbackGripperStatus)
    def _status_cb(self, msg):
        self.cur_status = msg

    def wait_for_connection(self, timeout=-1):
        rate = self.create_rate(0.1)
        rate.sleep()
        r = self.create_rate(30)
        start_time = self.get_clock().now()
        while rclpy.ok():
            if (timeout >= 0. and self.get_clock().now() - start_time > timeout):
                return False
            if self.cur_status is not None:
                return True
            r.sleep()
        return False

    def is_ready(self):
        return self.cur_status.gSTA == 3 and self.cur_status.gACT == 1

    def is_reset(self):
        return self.cur_status.gSTA == 0 or self.cur_status.gACT == 0

    def is_moving(self):
        return self.cur_status.gGTO == 1 and self.cur_status.gOBJ == 0

    def is_stopped(self):
        return self.cur_status.gOBJ != 0

    def object_detected(self):
        return self.cur_status.gOBJ == 1 or self.cur_status.gOBJ == 2

    def get_fault_status(self):
        return self.cur_status.gFLT

    def get_pos(self):
        po = self.cur_status.gPO
        return np.clip(0.087/(13.-230.)*(po-230.), 0, 0.087)

    def get_req_pos(self):
        pr = self.cur_status.gPR
        return np.clip(0.087/(13.-230.)*(pr-230.), 0, 0.087)

    def is_closed(self):
        return self.cur_status.gPO >= 230

    def is_opened(self):
        return self.cur_status.gPO <= 13

    # if timeout is negative, wait forever

    def wait_until_stopped(self, timeout=-1):
        r = self.create_rate(30)
        start_time = self.get_clock().now()
        while rclpy.ok() :
            if (timeout >= 0. and self.get_clock().now() - start_time > timeout) or self.is_reset():
                return False
            if self.is_stopped():
                return True
            r.sleep()
        return False

    def wait_until_moving(self, timeout=-1):
        r = self.create_rate(30)
        start_time = self.get_clock().now()
        while rclpy.ok:
            if (timeout >= 0. and self.get_clock().now() - start_time > timeout) or self.is_reset():
                return False
            if not self.is_stopped():
                return True
            r.sleep()
        return False

    def reset(self):
        cmd = outputMsg()
        cmd.rACT = 0
        self.cmd_pub.publish(cmd)

    def activate(self, timeout=-1):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rMOD = 0
        cmd.rGTO = 1
        cmd.rPR = 0
        cmd.rSP = 150
        cmd.rFR = 50
        self.cmd_pub.publish(cmd)
        r = self.create_rate(30)
        start_time = self.get_clock().now()
        while rclpy.ok():
            if timeout >= 0. and self.get_clock().now() - start_time > timeout:
                return False
            if self.is_ready():
                return True
            r.sleep()
        return False

    def auto_release(self):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rATR = 1
        self.cmd_pub.publish(cmd)

    ##
    # Actuate
    # @param press is the Gripper max relative pressure level request [0, 0.087]
    # @param rdel is the Gripper timeout / release delay [0.013, 0.100]
    # @param mrprl is the Gripper minimum relative pressure level request [30, 100]
    def goto(self, press, rdel, mrprl, block=False, timeout=1000):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = int(np.clip((13.-230.)/0.087 * press + 230., 0, 255))
        cmd.rSP = int(np.clip(255./(0.1-0.013) * (rdel-0.013), 0, 255))
        cmd.rFR = int(np.clip(255./(100.-30.) * (mrprl-30.), 0, 255))
        self.cmd_pub.publish(cmd)
        rate = self.create_rate(0.1)
        rate.sleep(0.1)
        if block:
            if not self.wait_until_moving(timeout):
                return False
            return self.wait_until_stopped(timeout)
        return True

    def stop(self, block=False, timeout=-1):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 0
        self.cmd_pub.publish(cmd)
        rate = self.create_rate(0.1)
        rate.sleep(0.1)
        if block:
            return self.wait_until_stopped(timeout)
        return True

    def open(self, rdel=0.1, mrprl=100, block=False, timeout=-1):
        if self.is_opened():            
            return True
        print("Gripper on")
        return self.goto(1.0, rdel, mrprl, block=block, timeout=timeout)

    def close(self, rdel=0.1, mrprl=100, block=False, timeout=-1):
        if self.is_closed():
            return True
        print("Gripper off")
        return self.goto(-1.0, rdel, mrprl, block=block, timeout=timeout)

    def callbackOn(self,req):
        print("[robotiq_vacuum_grippers_ctrl] turning gripper on ")  
        
        self.on_count=self.on_count+1  
        self.on_count_pub.publish(self.on_count)
        grip_try_count=0  
        release_try_count=0
        self.wait_for_connection()
        if self.is_reset():
            self.reset()
            self.activate()
    
        while(release_try_count<self.release_try_limit and self.close()==False):
            release_try_count=release_try_count+1
        
        while(grip_try_count<self.release_try_limit and self.open()==False):
            grip_try_count=grip_try_count+1   
        rate = self.create_timer(self.timeoutLarge)       
        rate.sleep()  
        response = SetBoolResponse()
        response.success=True
        return response

    def callbackOff(self,req):
        print("[robotiq_vacuum_grippers_ctrl] turning gripper off")
       
        self.off_count=self.off_count+1
        self.off_count_pub.publish(self.off_count)

    
        release_try_count=0    
        self.wait_for_connection()

        if self.is_reset():
            self.reset()
            self.activate()  
        self.close()   
        
        rate = self.create_timer(self.timeoutSmall)       
        rate.sleep()  
        while(release_try_count<self.release_try_limit and self.object_detected()):
            release_try_count=release_try_count+1
            self.close()
        
        if(release_try_count>=self.release_try_limit):
           print("[ERROR]release failed possible communication problem")
        response = SetBoolResponse()
        response.success=True
        return response
    

    def callbackGripperStatus(self,req):
        returnVal= SetBoolResponse()
        self.wait_for_connection()

        if self.is_reset():
            self.reset()
            self.activate() 
        print("getting gripper status")
        returnVal.success = self.object_detected() 
        print("[robotiq_vacuum_grippers_ctrl] Object detected "+str(self.object_detected()))
        return returnVal
    
def main(args = None):
    rclpy.init(args = args)
    
    gripper=RobotiqVGripper()    
    
    rclpy.spin(gripper)
if __name__ == '__main__':
    main()
