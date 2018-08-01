#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport rospy
@pyimport tf

@rosimport moveit_msgs.msg: Grasp, PlaceLocation

rostypegen()

using moveit_msgs.msg

function main()
    init_node("pick_n_place_based_on_target_tf")
    support_surface_name = get_param("~support_surface_name", "box_link")
    pick_link = get_param("~pick_link", "detected_object")
    place_link = get_param("~place_link", "target_place")
    arm_group_name = get_param("~arm_group_name", "arm")
    base_link = get_param("~base_link", "base_link")
    ee_link = get_param("~ee_link", "tool0")

    listener = tf.TransformListener()

    robot_commander = moveit_commander.RobotCommander()
    scene_interface = moveit_commander.PlanningSceneInterface()
    arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)
    arm_move_group[:set_planning_time](60.0)
    arm_move_group[:set_num_planning_attempts](20)
    arm_move_group[:set_support_surface_name](support_surface_name)
    println(arm_move_group[:get_planning_time]())


    while ! is_shutdown()
        try
            pick_trans, pick_rot = listener[:lookupTransform](base_link, pick_link, rospy.Time(0))
            place_trans, place_rot = listener[:lookupTransform](base_link, place_link, rospy.Time(0))

            grasp = Grasp()

            # Grasp pose
            grasp.grasp_pose.header.frame_id = ee_link
            grasp.grasp_pose.pose.position.x = pick_trans[1]# - 0.06
            grasp.grasp_pose.pose.position.y = pick_trans[2]
            grasp.grasp_pose.pose.position.z = pick_trans[3]
            grasp.grasp_pose.pose.orientation.x = pick_rot[1]
            grasp.grasp_pose.pose.orientation.y = pick_rot[2]
            grasp.grasp_pose.pose.orientation.z = pick_rot[3]
            grasp.grasp_pose.pose.orientation.w = pick_rot[4]

            # Pre-grasp pose (approach)
            grasp.pre_grasp_approach.direction.header.frame_id = ee_link
            grasp.pre_grasp_approach.direction.vector.x = 1.0
            grasp.pre_grasp_approach.min_distance = 0.095
            grasp.pre_grasp_approach.desired_distance = 0.115

            # Post-grasp pose (retreat)
            grasp.post_grasp_retreat.direction.header.frame_id = ee_link
            grasp.post_grasp_retreat.direction.vector.z = 1.0
            grasp.post_grasp_retreat.min_distance = 0.1
            grasp.post_grasp_retreat.desired_distance = 0.25

            # TODO openGripper function here

            # TODO closeGripper function here
            arm_move_group[:pick]("detected_object", grasp)

            # Place location
            place_location = PlaceLocation()
            place_location.place_pose.header.frame_id = ee_link
            #place_location.place_pose.pose.orientation = ...
            place_location.place_pose.pose.position.x = place_trans[1]
            place_location.place_pose.pose.position.y = place_trans[2]
            place_location.place_pose.pose.position.z = place_trans[3]

            # Pre-place pose (approach)
            place_location.pre_place_approach.direction.header.frame_id = ee_link
            place_location.pre_place_approach.direction.vector.z = -1.0
            place_location.pre_place_approach.min_distance = 0.095
            place_location.pre_place_approach.desired_distance = 0.115

            # Post-place pose (retreat)
            place_location.post_place_retreat.direction.header.frame_id = ee_link
            place_location.post_place_retreat.direction.vector.y = -1.0
            place_location.post_place_retreat.min_distance = 0.1
            place_location.post_place_retreat.desired_distance = 0.25

            # TODO openGripper function here

            println(pick_trans)
            break
         catch e
            println(e)
         end
    end
end

main()
