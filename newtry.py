import intera_interface
import rospy
import copy
import math 

from geometry_msgs.msg import Pose, Point, Quaternion

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None

g_image_path = "FILL THIS TO BOBS PICTURE PATH"

def init():
    global g_limb, g_orientation_hand_down, g_position_neutral, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q

    #Set up arm stuff
    rospy.init_node('cairo_sawyer_ik_example')
    g_limb = intera_interface.Limb('right')
    gripper = intera_interface.Gripper()

    # Straight down and 'neutral' position
    g_orientation_hand_down = Quaternion()
    g_orientation_hand_down.x = 0.704238785359
    g_orientation_hand_down.y = 0.709956638597
    g_orientation_hand_down.z = -0.00229009932359
    g_orientation_hand_down.w = 0.00201493272073
    g_position_neutral = Point()
    g_position_neutral.x = 0.45371551183
    g_position_neutral.y = 0.0663097073071
    g_position_neutral.z = 0.0271459370863

    #Marker position
    marker_q = Quaternion()
    marker_q.x = 0.704238785359
    marker_q.y = 0.709956638597
    marker_q.z = -0.00229009932359
    marker_q.w = 0.00201493272073
    marker_p = Point()
    marker_p.x = 0.525423244892
    marker_p.y = 0.254786824385
    marker_p.z = 0.0125670410943

    #Square position
    square_q = Quaternion()
    square_q.x = 0.704238785359
    square_q.y = 0.709956638597
    square_q.z = -0.00229009932359
    square_q.w = 0.00201493272073
    square_p = Point()
    square_p.x = 0.53430244888
    square_p.y = -0.152176453277
    square_p.z = 0.1125670410943


#Takes in POSE, SPEED, AND TIMEOUT, moves arm to that pose
def move_to(pose, speed, to):
    global g_limb
    angle = g_limb.ik_request(pose, "right_hand")
    g_limb.set_joint_position_speed(speed)
    g_limb.move_to_joint_positions(angle, timeout=to)

def rotate(degrees):
    #https://answers.ros.org/question/247808/tf-api-axesorder-of-rotation/
    roll = degrees * (3.1415/180.0)
    roll = 0
    roll = -1.0*degrees * (3.1415/180.0)

    q = Quaternion



#Draws a square, from neutral pose
def draw_square():
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    
    rospy.sleep(2)
    gripper.open()

    marker_pose = Pose()
    marker_pose.position = marker_p
    marker_pose.orientation = marker_q
    square_pose = Pose()
    square_pose.position = square_p
    square_pose.orientation = square_q

    move_to(marker_pose, 0.3, 5)
    rospy.sleep(2.0)
    gripper.close()
    rospy.sleep(2)

    marker_pose.position.z += 0.1
    move_to(marker_pose, 0.2, 5)

    move_to(square_pose, 0.3, 5)

    square_pose.position.z = 0.00215987405556
    move_to(square_pose, 0.1, 2)

    square_pose.position.x += 0.1
    move_to(square_pose, 0.075, 2)

    # rotate

    move_to(square_pose, 0.075, 2)

    square_pose.position.y += 0.1
    move_to(square_pose, 0.075, 2)

    square_pose.position.x -= 0.1
    move_to(square_pose, 0.075, 2)

    square_pose.position.y -= 0.1
    move_to(square_pose, 0.075, 2)

    move_to(marker_pose, 0.2, 5)

    marker_pose.position.z -= 0.1
    move_to(marker_pose, 0.2, 5)
    
    gripper.open()

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

    for_range = 16
    mover = math.pi / for_range
    movei = 0
    for i in range(0, for_range):
	    circle_pose.position.x += 0.025*(math.sin((math.pi*(i+1)) / for_range))
	    circle_pose.position.y += 0.025*(math.cos((math.pi*(i+1))/ for_range))
	    move_to(circle_pose, 0.075, 5)

def display_image():
    global g_image_path
    head_display = intera_interface.HeadDisplay()
    head_display.display_image(g_image_path)

def main():
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    init()

    # Move the arm to its neutral position, then draw square

    # display our lord and savior bob ross
    display_image()
    # Move the arm to its neutral position, then draw square
    g_limb.move_to_neutral()
    gripper.open()
    draw_square()
    # draw_semi_circle()
    g_limb.move_to_neutral()
    gripper.open()


if __name__ == "__main__":
    main()


#Information prints
#rospy.loginfo("Old Hand Pose:\n %s" % str(g_limb._tip_states.states[0].pose))
#rospy.loginfo("Old Joint Angles:\n %s" % str(g_limb.joint_angles()))
