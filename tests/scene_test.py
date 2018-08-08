#!/usr/bin/env python
import tf
import sys
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, PlaceLocation, PickupAction

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("scene_test")
scene = moveit_commander.PlanningSceneInterface()

listener = tf.TransformListener()
arm_move_group = moveit_commander.MoveGroupCommander("arm")
arm_move_group.set_planning_time(10.0)
arm_move_group.set_num_planning_attempts(10)
print arm_move_group.get_current_pose()

sac = actionlib.SimpleActionClient("/pickup", PickupAction)
sac.wait_for_server()
grasp = Grasp()
# Grasp pose
grasp.id = "detected_object"
grasp.grasp_pose.header.frame_id = "world"
grasp.grasp_pose.header.stamp = rospy.Time.now()
grasp.grasp_pose.pose.position.x = 0.0502923064626#pick_trans[1]# - 0.06
grasp.grasp_pose.pose.position.y = 0.386385891992#pick_trans[2]
grasp.grasp_pose.pose.position.z = 1.32695679147#pick_trans[3]
grasp.grasp_pose.pose.orientation.x = 0.798922350469#pick_rot[1]
grasp.grasp_pose.pose.orientation.y = -0.448319626964#pick_rot[2]
grasp.grasp_pose.pose.orientation.z = 0.239854921699#pick_rot[3]
grasp.grasp_pose.pose.orientation.w = 0.32125100239#pick_rot[4]
# Pre-grasp pose (approach)
grasp.pre_grasp_approach.direction.header.frame_id = "wrist_3_link"
grasp.pre_grasp_approach.direction.header.stamp = rospy.Time.now()
grasp.pre_grasp_approach.direction.vector.x = 1.0
grasp.pre_grasp_approach.min_distance = 0.095
grasp.pre_grasp_approach.desired_distance = 0.115
# Post-grasp pose (retreat)
grasp.post_grasp_retreat.direction.header.frame_id = "wrist_3_link"
grasp.post_grasp_retreat.direction.header.stamp = rospy.Time.now()
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

scene.remove_world_object("detected_object")
rospy.sleep(2)
p = PoseStamped()
p.header.frame_id = "world"
p.pose.position.x = -0.12
p.pose.position.y = 0.31
p.pose.position.z = 1.4
p.pose.orientation.w = 1.0
scene.add_box("detected_object", p, (0.05, 0.05, 0.05))

rospy.sleep(2)

print "Trying to use pick with movegroup"

arm_move_group.pick("detected_object", grasp)

# Place location
place_location = PlaceLocation()
place_location.place_pose.header.frame_id = "world"
place_location.place_pose.pose.position.x = -0.112323057508
place_location.place_pose.pose.position.y = 0.0850212109526
place_location.place_pose.pose.position.z = 1.46922744563
place_location.place_pose.pose.orientation.x = -0.497953660189
place_location.place_pose.pose.orientation.y = 0.497638795266
place_location.place_pose.pose.orientation.z = 0.50187777793
place_location.place_pose.pose.orientation.w = 0.502510176783

# Pre-place pose (approach)
place_location.pre_place_approach.direction.header.frame_id = "wrist_3_link"
place_location.pre_place_approach.direction.vector.z = -1.0
place_location.pre_place_approach.min_distance = 0.095
place_location.pre_place_approach.desired_distance = 0.115

# Post-place pose (retreat)
place_location.post_place_retreat.direction.header.frame_id = "wrist_3_link"
place_location.post_place_retreat.direction.vector.y = -1.0
place_location.post_place_retreat.min_distance = 0.1
place_location.post_place_retreat.desired_distance = 0.25

# Gripper release pose
place_location.post_place_posture.joint_names = finger_joint_names
place_location.post_place_posture.points = [JointTrajectoryPoint()]
place_location.post_place_posture.points[0].positions = [0.02, 0.02]
place_location.allowed_touch_objects = ["detected_object"]

print "Trying to use place with movegroup"

arm_move_group.place("detected_object", place_location)

