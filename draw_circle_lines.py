def draw_circle_lines():
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    
    rospy.sleep(2)
    gripper.open()

    marker_pose = Pose()
    marker_pose.position = marker_p
    marker_pose.orientation = marker_q

    fancy_pose = Pose()
    fancy_pose.position = square_p
    fancy_pose.orientation = square_q

    move_to(marker_pose, 0.3, 5)
    rospy.sleep(2.0)
    gripper.close()
    rospy.sleep(2)

    marker_pose.position.z += 0.1
    move_to(marker_pose, 0.2, 5)
    rospy.sleep(1)
    fancy_pose.position.z +=0.1
    move_to(fancy_pose, 0.3, 5)

    rad = 0.075
    notches = 32
    inc = 11
    index = 0
    cur_pose = Pose()
    cur_pose.orientation = fancy_pose.orientation
    cur_pose.position = fancy_pose.position
    counter = 0

    while (True):
        # calculate new position
        cur_pose.position.x = fancy_pose.position.x + rad*(math.sin(index*((2.0*math.pi)/notches)))
        cur_pose.position.y = fancy_pose.position.y + rad*(math.cos(index*((2.0*math.pi)/notches)))
        cur_pose.position.z = cur_pose.position.z
        # move to cur pose
        move_to(cur_pose, 0.3, 5)
        # if first time through subract 0.1 from z
        if counter == 0:
            cur_pose.position.z -= 0.1
            move_to(cur_pose, 0.3, 5)
        if counter > notches:
            break
        # increment index to next position
        index += inc
        # if index > notches mod by inc
        if (index > notches):
            index = index%notches
        # keep track of which position were in
        counter += 1
