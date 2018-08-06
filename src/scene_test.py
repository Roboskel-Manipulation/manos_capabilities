#!/usr/bin/env python
import tf
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, PlaceLocation, PickupAction

rospy.init_node("scene_test")
scene = moveit_commander.PlanningSceneInterface()

listener = tf.TransformListener()
arm_move_group = moveit_commander.MoveGroupCommander("arm")
gripper_move_group = moveit_commander.MoveGroupCommander("gripper")

sac = actionlib.SimpleActionClient("/pickup", PickupAction)
sac.wait_for_server()

rospy.sleep(2)

p = PoseStamped()
p.header.frame_id = "base_link"
p.pose.position.x = 0
p.pose.position.y = 0.5
p.pose.position.z = 0.44
scene.add_box("detected_object", p, (0.1, 0.1, 0.1))

rospy.sleep(2)

print "Trying to use pick with movegroup"

arm_move_group.pick("detected_object")

rospy.sleep(2)

print "Trying to use pick with the action server"

grasp = Grasp()
# Grasp pose
grasp.grasp_pose.header.frame_id = "base_link"
grasp.grasp_pose.header.stamp = rospy.Time(0)
grasp.grasp_pose.pose.position.x = 0#pick_trans[1]# - 0.06
grasp.grasp_pose.pose.position.y = 0.5#pick_trans[2]
grasp.grasp_pose.pose.position.z = 0.44#pick_trans[3]
grasp.grasp_pose.pose.orientation.x = 0#pick_rot[1]
grasp.grasp_pose.pose.orientation.y = 0#pick_rot[2]
grasp.grasp_pose.pose.orientation.z = 0#pick_rot[3]
grasp.grasp_pose.pose.orientation.w = 1#pick_rot[4]
# Pre-grasp pose (approach)
grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
grasp.pre_grasp_approach.direction.header.stamp = rospy.Time(0)
grasp.pre_grasp_approach.direction.vector.x = 1.0
grasp.pre_grasp_approach.min_distance = 0.095
grasp.pre_grasp_approach.desired_distance = 0.115
# Post-grasp pose (retreat)
grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
grasp.post_grasp_retreat.direction.header.stamp = rospy.Time(0)
grasp.post_grasp_retreat.direction.vector.z = 1.0
grasp.post_grasp_retreat.min_distance = 0.1
grasp.post_grasp_retreat.desired_distance = 0.25
# Pre-grasp gripper joint state
finger_joint_names = ["pg70_finger1_joint", "pg70_finger2_joint"]
grasp.pre_grasp_posture.joint_names = finger_joint_names
grasp.pre_grasp_posture.points = [JointTrajectoryPoint()]
grasp.pre_grasp_posture.points[0].positions = [0.02, 0.02]
# Post-grasp gripper joint state
grasp.grasp_posture.joint_names = finger_joint_names
grasp.grasp_posture.points = [JointTrajectoryPoint()]
grasp.grasp_posture.points[0].positions = [0.001, 0.001]
grasp.allowed_touch_objects = ["detected_object"]


pickup = PickupAction()
#println(fieldnames(pickup.action_goal.goal))
pickup.action_goal.goal.target_name = "detected_object"
pickup.action_goal.goal.group_name = "gripper"
pickup.action_goal.goal.end_effector = "gripper"
pickup.action_goal.goal.possible_grasps = [grasp]
pickup.action_goal.goal.allowed_touch_objects = ["all"]
pickup.action_goal.goal.allowed_planning_time = 40.0

sac.send_goal(pickup.action_goal.goal)
print "\n\n\nSent pickup goal!\n\n\n"
sac.wait_for_result(rospy.Duration.from_sec(45))
print sac.get_result()

arm_move_group.set_position_target((0, 0.5, 1.29), "ee_link")
arm_move_group.go()
