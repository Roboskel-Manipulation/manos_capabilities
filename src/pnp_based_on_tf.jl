#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport actionlib
@pyimport rospy
@pyimport tf

@rosimport moveit_msgs.msg: Grasp, PlaceLocation, PickupAction
@rosimport geometry_msgs.msg: PoseStamped

rostypegen()

using geometry_msgs.msg
using moveit_msgs.msg

function straight2Action(base_link, ee_link, arm_group_name, sac, pick_trans, pick_rot)

    println("I am inside the straight2Action function!")
    grasp = Grasp()
    # Grasp pose
    grasp.grasp_pose.header.frame_id = base_link
    grasp.grasp_pose.header.stamp = Time(0)
    grasp.grasp_pose.pose.position.x = 0#pick_trans[1]# - 0.06
    grasp.grasp_pose.pose.position.y = 0.5#pick_trans[2]
    grasp.grasp_pose.pose.position.z = 0.44#pick_trans[3]
    grasp.grasp_pose.pose.orientation.x = 0#pick_rot[1]
    grasp.grasp_pose.pose.orientation.y = 0#pick_rot[2]
    grasp.grasp_pose.pose.orientation.z = 0#pick_rot[3]
    grasp.grasp_pose.pose.orientation.w = 1#pick_rot[4]
    # Pre-grasp pose (approach)
    grasp.pre_grasp_approach.direction.header.frame_id = base_link
    grasp.pre_grasp_approach.direction.header.stamp = Time(0)
    grasp.pre_grasp_approach.direction.vector.x = 1.0
    grasp.pre_grasp_approach.min_distance = 0.095
    grasp.pre_grasp_approach.desired_distance = 0.115
    # Post-grasp pose (retreat)
    grasp.post_grasp_retreat.direction.header.frame_id = base_link
    grasp.post_grasp_retreat.direction.header.stamp = Time(0)
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.1
    grasp.post_grasp_retreat.desired_distance = 0.25
    # Pre-grasp gripper joint state
    finger_joint_names = ["pg70_finger1_joint", "pg70_finger2_joint"]
    grasp.pre_grasp_posture.joint_names = finger_joint_names
    grasp.pre_grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
    grasp.pre_grasp_posture.points[1].positions = [0.02, 0.02]
    # Post-grasp gripper joint state
    grasp.grasp_posture.joint_names = finger_joint_names
    grasp.grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
    grasp.grasp_posture.points[1].positions = [0.001, 0.001]
    grasp.allowed_touch_objects = ["detected_object"]


    pickup = PickupAction()
    #println(fieldnames(pickup.action_goal.goal))
    pickup.action_goal.goal.target_name = "detected_object"
    pickup.action_goal.goal.group_name = "gripper"
    pickup.action_goal.goal.end_effector = "gripper"
    pickup.action_goal.goal.possible_grasps = [grasp]
    pickup.action_goal.goal.allowed_touch_objects = ["all"]
    pickup.action_goal.goal.allowed_planning_time = 40.0

    sac[:send_goal](pickup.action_goal.goal)
    println("\n\n\nSent pickup goal!\n\n\n")
    sac[:wait_for_result](rospy.Duration[:from_sec](45))
    println(sac[:get_result]())
end

function main()
    init_node("pick_n_place_based_on_target_tf")
    support_surface_name = get_param("~support_surface_name", "box_link")
    pick_link = get_param("~pick_link", "detected_object")
    place_link = get_param("~place_link", "target_place")
    arm_group_name = get_param("~arm_group_name", "arm")
    base_link = get_param("~base_link", "base_link")
    world_link = get_param("~world_link", "world")
    ee_link = get_param("~ee_link", "ee_link")

    listener = tf.TransformListener()

    #moveit_commander.roscpp_initialize(ARGS)
    robot_commander = moveit_commander.RobotCommander()
    scene_interface = moveit_commander.PlanningSceneInterface()
    arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)
    gripper_move_group = moveit_commander.MoveGroupCommander("gripper")
    # arm_move_group[:set_planning_time](60.0)
    # arm_move_group[:set_num_planning_attempts](4)
    # arm_move_group[:set_support_surface_name](support_surface_name)

    sleep(2)

    scene_interface[:remove_world_object]("detected_object")
    # add an object to be grasped
    p = PoseStamped()
    p.header.stamp = rospy.Time(0)
    p.header.frame_id = "base_link"
    p.pose.position.x = 0
    p.pose.position.y = 0.5
    p.pose.position.z = 0.44
    p.pose.orientation.w = 1.0
    # MAJOR BUG: the add_box function does not seem to work on Julia!
    scene_interface[:add_box]("detected_object", p, (0.1, 0.1, 0.1))

    sac = actionlib.SimpleActionClient("/pickup", PickupAction)
    sac[:wait_for_server]()
    println("Connected to pickup action server!")

    println(scene_interface[:get_known_object_names]())
    println(robot_commander[:get_planning_frame]())

    while ! is_shutdown()
        try
            pick_trans, pick_rot = listener[:lookupTransform](world_link, pick_link, rospy.Time(0))
            #place_trans, place_rot = listener[:lookupTransform](world_link, place_link, rospy.Time(0))

            straight2Action(base_link, ee_link, arm_group_name, sac, pick_trans, pick_rot)

            # grasp = Grasp()

            # # Grasp pose
            # grasp.grasp_pose.header.frame_id = base_link
            # grasp.grasp_pose.header.stamp = Time(0)
            # grasp.grasp_pose.pose.position.x = 0#pick_trans[1]# - 0.06
            # grasp.grasp_pose.pose.position.y = 0.5#pick_trans[2]
            # grasp.grasp_pose.pose.position.z = 0.44#pick_trans[3]
            # grasp.grasp_pose.pose.orientation.x = 0.91#pick_rot[1]
            # grasp.grasp_pose.pose.orientation.y = 0.24#pick_rot[2]
            # grasp.grasp_pose.pose.orientation.z = -0.17#pick_rot[3]
            # grasp.grasp_pose.pose.orientation.w = -0.27#pick_rot[4]

            # # Pre-grasp pose (approach)
            # grasp.pre_grasp_approach.direction.header.frame_id = ee_link
            # grasp.pre_grasp_approach.direction.header.stamp = Time(0)
            # grasp.pre_grasp_approach.direction.vector.x = 1.0
            # grasp.pre_grasp_approach.min_distance = 0.095
            # grasp.pre_grasp_approach.desired_distance = 0.115

            # # Post-grasp pose (retreat)
            # grasp.post_grasp_retreat.direction.header.frame_id = ee_link
            # grasp.post_grasp_retreat.direction.header.stamp = Time(0)
            # grasp.post_grasp_retreat.direction.vector.z = 1.0
            # grasp.post_grasp_retreat.min_distance = 0.1
            # grasp.post_grasp_retreat.desired_distance = 0.25

            # # Pre-grasp gripper joint state
            # finger_joint_names = ["pg70_finger1_joint", "pg70_finger2_joint"]
            # grasp.pre_grasp_posture.joint_names = finger_joint_names
            # grasp.pre_grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
            # grasp.pre_grasp_posture.points[1].positions = [0.02, 0.02]

            # # Post-grasp gripper joint state
            # grasp.grasp_posture.joint_names = finger_joint_names
            # grasp.grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
            # grasp.grasp_posture.points[1].positions = [0.001, 0.001]

            # grasp.allowed_touch_objects = ["detected_object"]

            # g2 = grasp
            # g2.grasp_pose.pose.position.x = 0.0#pick_trans[1]# - 0.06
            # g2.grasp_pose.pose.position.y = 0#pick_trans[2]
            # g2.grasp_pose.pose.position.z = 0.9#pick_trans[3]

            # g3 = grasp
            # g3.grasp_pose.pose.position.x = 0.0#pick_trans[1]# - 0.06
            # g3.grasp_pose.pose.position.y = 0#pick_trans[2]
            # g3.grasp_pose.pose.position.z = 1.0#pick_trans[3]

            # g4 = grasp
            # g4.grasp_pose.pose.position.x = 0.0#pick_trans[1]# - 0.06
            # g4.grasp_pose.pose.position.y = 0#pick_trans[2]
            # g4.grasp_pose.pose.position.z = 0.7#pick_trans[3]

            # g5 = grasp
            # g5.grasp_pose.pose.position.x = 0.5
            # g5.grasp_pose.pose.position.y = 0.1
            # g5.grasp_pose.pose.position.z = 0.5

            # g6 = grasp
            # g6.grasp_pose.pose.position.x = 0.5
            # g6.grasp_pose.pose.position.y = 0.2
            # g6.grasp_pose.pose.position.z = 0.5

            # grasps = [grasp, g2, g3, g4, g5, g6]

            # sleep(2)

            # #arm_move_group[:pick]("detected_object", grasps)
            #arm_move_group[:pick]("detected_object")

            # # Place location
            # place_location = PlaceLocation()
            # place_location.place_pose.header.frame_id = base_link
            # #place_location.place_pose.pose.orientation = ...
            # #place_location.place_pose.pose.position.x = place_trans[1]
            # #place_location.place_pose.pose.position.y = place_trans[2]
            # #place_location.place_pose.pose.position.z = place_trans[3]

            # # Pre-place pose (approach)
            # place_location.pre_place_approach.direction.header.frame_id = base_link
            # place_location.pre_place_approach.direction.vector.z = -1.0
            # place_location.pre_place_approach.min_distance = 0.095
            # place_location.pre_place_approach.desired_distance = 0.115

            # # Post-place pose (retreat)
            # place_location.post_place_retreat.direction.header.frame_id = base_link
            # place_location.post_place_retreat.direction.vector.y = -1.0
            # place_location.post_place_retreat.min_distance = 0.1
            # place_location.post_place_retreat.desired_distance = 0.25

            # # Gripper release pose
            # place_location.post_place_posture.joint_names = finger_joint_names
            # place_location.post_place_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
            # place_location.post_place_posture.points[1].positions = [0.02, 0.02]

            # place_location.allowed_touch_objects = ["detected_object"]

            # #arm_move_group[:place]("detected_object", place_location)

            println(pick_trans)
            break
         catch e
            println(e)
         end
    end
end

main()
