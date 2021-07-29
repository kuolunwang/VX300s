#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from vx300s_bringup.srv import *
from sensor_msgs.msg import JointState

import tf
import tf.transformations as tfm

class vx300s():
    def __init__(self):

        # Service
        rospy.Service("/vx300s/go_home", Trigger, self.vx300s_home)
        rospy.Service("/vx300s/go_sleep", Trigger, self.vx300s_sleep)
        rospy.Service("/vx300s/go_pose", ee_pose, self.vx300s_ee_pose)
        rospy.Service("/vx300s/gripper_open", Trigger, self.vx300s_open)
        rospy.Service("/vx300s/gripper_close", Trigger, self.vx300s_close)
        rospy.Service("/vx300s/check_grasped", Trigger, self.vx300s_check)

        # vx300s setup
        robot = InterbotixManipulatorXS("vx300s", "arm", "gripper")

        self.arm = robot.arm
        self.gripper = robot.gripper

        self.init()

    def init(self):

        self.gripper.open(2.0)
        self.arm.go_to_home_pose()
        rospy.loginfo("initial already!")

    def vx300s_home(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_home_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_sleep(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_sleep_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_check(self, req):
        
        res = TriggerResponse()

        try:
            joint_info = rospy.wait_for_message('/vx300s/joint_states', JointState)
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)

        if(joint_info.position[6] <= 1.39 and joint_info.position[6] >= -0.42):
            res.success = True
            rospy.loginfo("grasped object")
        else:
            res.success = False
            rospy.loginfo("no object grasped")

        return res

    def vx300s_open(self, req):

        res = TriggerResponse()

        try:
            self.gripper.open(2.0)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_close(self, req):

        res = TriggerResponse()

        try:
            self.gripper.close(2.0)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_ee_pose(self, req):

        res = ee_poseResponse()

        try:
            x = req.target_pose.position.x
            y = req.target_pose.position.y
            z = req.target_pose.position.z
            ox = req.target_pose.orientation.x
            oy = req.target_pose.orientation.y
            oz = req.target_pose.orientation.z
            ow = req.target_pose.orientation.w
            roll, pitch, yaw = tfm.euler_from_quaternion([ox, oy, oz, ow])
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
            res.result = "success"
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.result = "Fail"
            print("Service call failed: %s"%e)
        
        return res

if __name__=='__main__':

    VX300s = vx300s()
    rospy.spin()