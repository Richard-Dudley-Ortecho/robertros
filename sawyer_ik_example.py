import intera_interface
import rospy
import copy 

from geometry_msgs.msg import Pose, Point, Quaternion

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None

def init():
    global g_limb, g_orientation_hand_down, g_position_neutral, pos, posp, gripper
    rospy.init_node('cairo_sawyer_ik_example')
    g_limb = intera_interface.Limb('right')
    gripper = intera_interface.Gripper()
    # This quaternion will have the hand face straight down (ideal for picking tasks)
    g_orientation_hand_down = Quaternion()
    g_orientation_hand_down.x = 0.704238785359
    g_orientation_hand_down.y = 0.709956638597
    g_orientation_hand_down.z = -0.00229009932359
    g_orientation_hand_down.w = 0.00201493272073

    # g_orientation_hand_down.x = 0.65759208006
    # g_orientation_hand_down.y = 0.00507142331692
    # g_orientation_hand_down.z = 0.753339706619
    # g_orientation_hand_down.w = -0.00512087278968


    # This is the default neutral position for the robot's hand (no guarantee this will move the joints to neutral though)
    g_position_neutral = Point()

    # g_position_neutral.x = 0.449559195663
    # g_position_neutral.y = 0.16070379419
    # g_position_neutral.z = 0.212938808947
    g_position_neutral.x = 0.45371551183
    g_position_neutral.y = 0.0663097073071
    g_position_neutral.z = 0.0271459370863



    pos = Quaternion()
    pos.x = 0.704238785359
    pos.y = 0.709956638597
    pos.z = -0.00229009932359
    pos.w = 0.00201493272073

    posp = Point()
    posp.x = 0.45371551183
    posp.y = 0.2663097073071
    posp.z = 0.0401459370863


def main():
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    init()

    # Move the arm to its neutral position
    g_limb.move_to_neutral()

    rospy.loginfo("Old Hand Pose:\n %s" % str(g_limb._tip_states.states[0].pose))
    rospy.loginfo("Old Joint Angles:\n %s" % str(g_limb.joint_angles()))

    # Create a new pose (Position and Orientation) to solve for
    target_pose = Pose()
    pose2 = Pose()
    target_pose.position = copy.deepcopy(g_position_neutral)
    target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
    pose2.position = posp
    pose2.orientation = pos 
    # target_pose.position.x += 0.2 # Add 20cm to the x axis position of the hand

    # Call the IK service to solve for joint angles for the desired pose
    target_joint_angles = g_limb.ik_request(target_pose, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if target_joint_angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(target_pose))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.3)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(target_joint_angles, timeout=2)

    
    
    
    
    angles = g_limb.ik_request(pose2, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(pose2))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.3)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(angles, timeout=2)

    # Find the new coordinates of the hand and the angles the motors are currently at
    new_hand_pose = copy.deepcopy(g_limb._tip_states.states[0].pose)
    new_angles = g_limb.joint_angles()

    gripper.close()
    rospy.sleep(1.0)
    pose2.position.x += 0.1

    angles = g_limb.ik_request(pose2, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(pose2))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.1)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(angles, timeout=2)
    
    
    pose2.position.y += 0.1

    angles = g_limb.ik_request(pose2, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(pose2))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.1)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(angles, timeout=2)
    
    
    pose2.position.x -= 0.1

    angles = g_limb.ik_request(pose2, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(pose2))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.1)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(angles, timeout=2)
    
    pose2.position.y -= 0.1

    angles = g_limb.ik_request(pose2, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(pose2))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.1)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(angles, timeout=2)
    
    
    
    gripper.open()
    # gripper.close()
    # g_limb.gripper_open()
    rospy.loginfo("New Hand Pose:\n %s" % str(new_hand_pose))
    rospy.loginfo("Target Joint Angles:\n %s" % str(target_joint_angles))
    rospy.loginfo("New Joint Angles:\n %s" % str(new_angles))

if __name__ == "__main__":
    main()