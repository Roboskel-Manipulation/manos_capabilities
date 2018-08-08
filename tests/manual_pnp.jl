#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport rospy
@pyimport tf

@rosimport geometry_msgs.msg: PoseStamped
@rosimport schunk_pg70.srv: set_position

rostypegen()

using geometry_msgs.msg
using schunk_pg70.srv

function main()
    init_node("approach_tf")
    pick_link = get_param("~pick_link", "detected_object")
    place_link = get_param("~place_link", "target_place")
    arm_group_name = get_param("~arm_group_name", "arm")
    base_link = get_param("~base_link", "base_link")

    listener = tf.TransformListener()

    moveit_commander.roscpp_initialize(ARGS)
    robot_commander = moveit_commander.RobotCommander()
    scene_interface = moveit_commander.PlanningSceneInterface()
    arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)

    arm_move_group[:set_planning_time](10.0)
    arm_move_group[:set_num_planning_attempts](10)

    eef_link = arm_move_group[:get_end_effector_link]()
    print(eef_link)

    planning_frame = robot_commander[:get_planning_frame]()
    println(planning_frame)

    finished = false

    gripper_service = ServiceProxy("schunk_pg70/set_position", set_position)
    wait_for_service("schunk_pg70/set_position")
    gripper_service(set_positionRequest(60, 80, 80))
    while ! is_shutdown() && !finished
        try
            pick_trans, pick_rot = listener[:lookupTransform](planning_frame, pick_link, rospy.Time(0))
            place_trans, place_rot = listener[:lookupTransform](planning_frame, place_link, rospy.Time(0))

            println("Pick distant location")
            # Pick distant location
            p = PoseStamped()
            p.header.stamp = RobotOS.now()
            p.header.frame_id = planning_frame
            p.pose.position.x = pick_trans[1]
            p.pose.position.y = pick_trans[2]
            p.pose.position.z = pick_trans[3] + 0.4
            # Top-down orientation
            p.pose.orientation.x = -0.71171
            p.pose.orientation.y = -0.017323
            p.pose.orientation.z = 0.0038264
            p.pose.orientation.w = 0.70225
            arm_move_group[:set_pose_target](p)
            if arm_move_group[:go]()
                println(pick_trans)

                println("Pick approach location")
                # Pick approach
                p.pose.position.x = pick_trans[1]
                p.pose.position.y = pick_trans[2]
                p.pose.position.z = pick_trans[3] + 0.25
                arm_move_group[:set_pose_target](p)
                if arm_move_group[:go]()

                    println("Grip")
                    # Grip
                    if gripper_service(set_positionRequest(20, 80, 80)).goal_accepted
                        sleep(1)

                        println("Pick retreat location")
                        # Pick retreat location
                        p.pose.position.x = pick_trans[1]
                        p.pose.position.y = pick_trans[2]
                        p.pose.position.z = pick_trans[3] + 0.4
                        arm_move_group[:set_pose_target](p)
                        if arm_move_group[:go]()

                            println("Place distant location")
                            # Place distant location
                            p.pose.position.x = place_trans[1]
                            p.pose.position.y = place_trans[2]
                            p.pose.position.z = place_trans[3] + 0.4
                            arm_move_group[:set_pose_target](p)
                            if arm_move_group[:go]()

                                println("Place approach location")
                                # Place approach location
                                p.pose.position.x = place_trans[1]
                                p.pose.position.y = place_trans[2]
                                p.pose.position.z = place_trans[3] + 0.3
                                arm_move_group[:set_pose_target](p)
                                if arm_move_group[:go]()

                                    println("Ungrip")
                                    # Ungrip
                                    if gripper_service(set_positionRequest(60, 80, 80)).goal_accepted
                                        sleep(1)

                                        println("Place retreat location")
                                        # Place retreat location
                                        p.pose.position.x = place_trans[1]
                                        p.pose.position.y = place_trans[2]
                                        p.pose.position.z = place_trans[3] + 0.4
                                        arm_move_group[:set_pose_target](p)
                                        if arm_move_group[:go]()
                                            finished = true
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
            if !finished
                println("Resetting position due to failure")
                arm_move_group[:set_named_target]("up")
                arm_move_group[:go]()
            end
         catch e
            println(e)
         end
    end
end

main()
