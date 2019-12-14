import math
def draw_semi_circle():
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    
    rospy.sleep(2)
    gripper.open()

    marker_pose = Pose()
    marker_pose.position = marker_p
    marker_pose.orientation = marker_q
    circle_pose = Pose()
    circle_pose.position = square_p
    circle_pose.orientation = square_q

    move_to(marker_pose, 0.3, 5)
    rospy.sleep(2.0)
    gripper.close()

    marker_pose.position.z += 0.1
    move_to(marker_pose, 0.2, 5)

    move_to(circle_pose, 0.3, 5)

    circle_pose.position.z = 0.00215987405556
    move_to(circle_pose, 0.1, 2)

    for_range = 8
    for i in range(0, for_range):
	circle_pose.position.x += math.sin(i) / for_range
	circle_pose.position.y += math.cos(i) / for_range
	move_to(circle_pose, 0.1, 2)
