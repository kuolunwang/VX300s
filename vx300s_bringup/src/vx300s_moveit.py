#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from vx300s_bringup.srv import *
from std_srvs.srv import Trigger, TriggerResponse
import tf
import tf.transformations as tfm
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    self.listener = tf.TransformListener()

    rospy.Service("get_pose", cur_pose, self.get_pose)
    rospy.Service("go_pose", ee_pose, self.vx300s_ee_pose)
    rospy.Service("go_joint_pose", joint_pose, self.vx300s_joint_pose)
    rospy.Service("go_home", Trigger, self.vx300s_home)
    rospy.Service("go_sleep", Trigger, self.vx300s_sleep)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Interbotix
    ## arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Interbotix Arm:
    group_name = "interbotix_arm"
    self.group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    self.display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.group.get_planning_frame()
    print("============ Reference frame: %s" % self.planning_frame)

    # We can also print the name of the end-effector link for this group:
    self.eef_link = self.group.get_end_effector_link()
    print("============ End effector: %s" % self.eef_link)

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    print("============ Robot Groups: " + str(self.group_names))

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(self.robot.get_current_state())
    print("")

  def get_pose(self, req):

    res = cur_poseResponse()

    try:
        trans, rot = self.listener.lookupTransform("base_link", "vx300s/ee_gripper_link", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print("Service call failed: %s"%e)

    res.pose.position.x = trans[0]
    res.pose.position.y = trans[1]
    res.pose.position.z = trans[2]
    res.pose.orientation.x = rot[0]
    res.pose.orientation.y = rot[1]
    res.pose.orientation.z = rot[2]
    res.pose.orientation.w = rot[3]

    return res

  def vx300s_ee_pose(self, req):

    res = ee_poseResponse()

    try:
        self.go_to_pose_goal(req.target_pose)
        res.result = "success"
    except (rospy.ServiceException, rospy.ROSException) as e:
        res.result = "Fail"
        print("Service call failed: %s"%e)
    
    return res

  def vx300s_joint_pose(self, req):

    res = joint_poseResponse()

    try:
        self.go_to_joint_state(req.joint_value)
        res.result = "success"
    except (rospy.ServiceException, rospy.ROSException) as e:
        res.result = "Fail"
        print("Service call failed: %s"%e)

    return res

  def vx300s_home(self, req):

    res = TriggerResponse()

    try:
        self.go_to_joint_state([0.0, -0.9, 1.1, 0.0, -0.25, 0.0])
        res.success = True
    except (rospy.ServiceException, rospy.ROSException) as e:
        res.success = False
        print("Service call failed: %s"%e)
    
    return res

  def vx300s_sleep(self, req):

    res = TriggerResponse()

    try:
        self.go_to_joint_state([0, -1.85, 1.55, 0, 0.8, 0, 0])
        res.success = True
    except (rospy.ServiceException, rospy.ROSException) as e:
        res.success = False
        print("Service call failed: %s"%e)
    
    return res
    
  def go_to_pose_goal(self, pose_goal):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    print("============ Printing Pose Goal:\n" + str(pose_goal))
    self.group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    print(current_pose)
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_joint_state(self, joint_goal):
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^

    print("============ Printing Joint Goal: " + str(joint_goal))

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

if __name__=='__main__':

    rospy.init_node("vx300s_moveit_node", anonymous=False)
    VX300s = MoveGroupPythonIntefaceTutorial()
    
    rospy.spin()