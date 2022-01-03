#!/usr/bin/env python3

from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippers_robot_input as inputMsg
from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippers_robot_output as outputMsg
from robotiq_vacuum_grippers_control.srv import gripper_cmd,gripper_cmdResponse
 
import std_srvs
from std_srvs.srv import SetBool,SetBoolResponse
from std_srvs.srv import Empty,EmptyResponse
import rospy
import numpy as np
from std_msgs.msg import String
import roslib
roslib.load_manifest('robotiq_vacuum_grippers_control')
from std_msgs.msg import Int16


class RobotiqVGripper(object):
    def __init__(self):
        self.cur_status = None
        self.status_sub = rospy.Subscriber('RobotiqVacuumGrippersRobotInput', inputMsg,
                                           self._status_cb)
        self.cmd_pub = rospy.Publisher(
            'RobotiqVacuumGrippersRobotOutput', outputMsg,queue_size=10)
        self.release_try_limit = 10
        self.timeoutSmall = 0.05
        self.timeoutLarge = 1.5
        self.timeoutminimum=0.005
        self.on_count_pub = rospy.Publisher('/on_service_count', Int16, queue_size=10)
        self.off_count_pub = rospy.Publisher('/off_service_count', Int16, queue_size=10)
        self.on_count = 0
        self.off_count = 0
        serviceOn = rospy.Service('/on', std_srvs.srv.SetBool, self.callbackOn)
        serviceOff = rospy.Service('/off', std_srvs.srv.SetBool, self.callbackOff)    

        serviceObjectDetected = rospy.Service('/grip_status', std_srvs.srv.SetBool,self.callbackGripperStatus)
    def _status_cb(self, msg):
        self.cur_status = msg

    def wait_for_connection(self, timeout=-1):
        rospy.sleep(0.1)
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout):
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
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout) or self.is_reset():
                return False
            if self.is_stopped():
                return True
            r.sleep()
        return False

    def wait_until_moving(self, timeout=-1):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout) or self.is_reset():
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
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if timeout >= 0. and rospy.get_time() - start_time > timeout:
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
        rospy.sleep(0.1)
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
        rospy.sleep(0.1)
        if block:
            return self.wait_until_stopped(timeout)
        return True

    def open(self, rdel=0.1, mrprl=100, block=False, timeout=-1):
        if self.is_opened():            
            return True
        rospy.loginfo("Gripper on")
        return self.goto(1.0, rdel, mrprl, block=block, timeout=timeout)

    def close(self, rdel=0.1, mrprl=100, block=False, timeout=-1):
        if self.is_closed():
            return True
        rospy.loginfo("Gripper off")
        return self.goto(-1.0, rdel, mrprl, block=block, timeout=timeout)

    def callbackOn(self,req):
        rospy.loginfo("[robotiq_vacuum_grippers_ctrl] turning gripper on ")  
        
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
        rospy.sleep(self.timeoutLarge)  
        response = SetBoolResponse()
        response.success=True
        return response

    def callbackOff(self,req):
        rospy.loginfo("[robotiq_vacuum_grippers_ctrl] turning gripper off")
       
        self.off_count=self.off_count+1
        self.off_count_pub.publish(self.off_count)

    
        release_try_count=0    
        self.wait_for_connection()

        if self.is_reset():
            self.reset()
            self.activate()  
        self.close()   
        
        rospy.sleep(self.timeoutSmall)
        while(release_try_count<self.release_try_limit and self.object_detected()):
            release_try_count=release_try_count+1
            self.close()
        
        if(release_try_count>=self.release_try_limit):
           rospy.loginfo("[ERROR]release failed possible communication problem")
        response = SetBoolResponse()
        response.success=True
        return response
    

    def callbackGripperStatus(self,req):
        returnVal= SetBoolResponse()
        self.wait_for_connection()
        polling_timeout=0;
        if self.is_reset():
            self.reset()
            self.activate() 
        rospy.loginfo("getting gripper status")
        while(polling_timeout<self.release_try_limit):
            polling_timeout=polling_timeout+1
            returnVal.success = self.object_detected()
            if(returnVal.success):
                break

        rospy.loginfo("[robotiq_vacuum_grippers_ctrl] Object detected "+str(self.object_detected()))
        return returnVal
    
def main():
    rospy.init_node("robotiq_vacuum_grippers_ctrl")
    gripper=RobotiqVGripper()    
    
    rospy.spin()

if __name__ == '__main__':
    main()
