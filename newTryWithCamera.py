import intera_interface
import rospy
import copy
import math
import apriltag_ros.msg

from geometry_msgs.msg import Pose, Point, Quaternion

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None
board_width = 0.25
board_height = 0.25



def init():
    global g_limb, g_orientation_hand_down, g_position_neutral, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    global llc_p, llc_q
    global prev_pose, rotation
    global subscriber_pose
    global cam_pos

    #Set up arm stuff
    rospy.init_node('cairo_sawyer_ik_example')
    g_limb = intera_interface.Limb('right')


    subscriber_pose = rospy.Subscriber('/robot/limb/right/endpoint_state', Pose, callback_subscriber_pose)
    rospy.sleep(2)
    gripper = intera_interface.Gripper()

    #set up camera control
    rospy.init_node('tags')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback_cam_pos)

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

    prev_pose = Pose()
    prev_pose.orientation = g_orientation_hand_down
    prev_pose.position = g_position_neutral

    #Marker position
    marker_q = Quaternion()
    marker_q.x = 0.704238785359
    marker_q.y = 0.709956638597
    marker_q.z = -0.00229009932359
    marker_q.w = 0.00201493272073
    marker_p = Point()
        #these ducking positions better be meters or im out
    marker_p.x = g_position_neutral.x + (cam.base.x - cam.marker.x)
    marker_p.y = g_position_neutral.y + (cam.base.y - cam.marker.y)
        #plz test
    marker_p.z = 0.00
    # marker 0.0125670410943

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

    #Lower Left Corner of board
    llc_q = Quaternion()
    llc_q.x = 0.70423878535
    llc_q.y = 0.709956638597
    llc_q.z = -0.00229009932359
    llc_q.w = 0.00201493272073
    llc_p = Point()
    llc_p.x = 0.487424263171
    llc_p.y = -0.10
    llc_p.z = -0.05

#Takes in POSE, SPEED, AND TIMEOUT, moves arm to that pose

def callback_subscriber_pose(msg):
    global subscriber_pose_state
    # pose
    subscriber_pose_state.position = msg.pose.position
    # # quaternion
    subscriber_pose_state.orientation = msg.pose.orientation

def callback_cam_pos(msg):
    global cam_pos
    if len(msg.detections) != 3:
        print("There is not 3 tags")  
        return
    for i in range(0, len(msg.detections)):
        #i hope these return as float numbers and not std_msg::Float32
        if msg.detections[i].id[0] == 0:
            cam.base.x = msg.detections[i].pose.pose.pose.position.x
            cam.base.y = msg.detections[i].pose.pose.pose.position.y
            cam.base.y += .01 #cannot put tag right under arm, put it 10 cm back
        elif msg.detections[i].id[0] == 1:
            cam.marker.x = msg.detections[i].pose.pose.pose.position.x
            cam.marker.y = msg.detections[i].pose.pose.pose.position.y
        elif msg.detections[i].id[0] == 2:
            cam.board.x = msg.detections[i].pose.pose.pose.position.x
            cam.board.y = msg.detections[i].pose.pose.pose.position.y

def move_to(pose, speed, to):
    global g_limb, prev_pose
    x_dist = pose.position.x - prev_pose.position.x
    y_dist = pose.position.y - prev_pose.position.y
    ang = math.atan2(y_dist, x_dist)
    #rotate(ang)

    angle = g_limb.ik_request(pose, "right_hand")
    g_limb.set_joint_position_speed(speed)
    g_limb.move_to_joint_positions(angle, timeout=to)
    prev_pose = pose

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
        move_to(cur_pose, 0.15, 5)
        # if first time through subract 0.1 from z
        if counter == 0:
            cur_pose.position.z -= 0.21
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

from scipy import misc
from skimage import color, measure
import matplotlib.pyplot as plt
from skimage.draw import ellipse
from skimage.measure import find_contours, approximate_polygon, subdivide_polygon
#http://freeconnection.blogspot.com/2013/07/vectorize-image-with-python-scikit-image.html
def draw_image(image):
    global board_height, board_width

    fimg = misc.imread(image)
    gimg = color.colorconv.rgb2grey(fimg)
    contours = measure.find_contours(gimg, 0.8)
    contour = contours[0]
    new_s = contour.copy()
    appr_s = approximate_polygon(new_s, tolerance=0.5)
    xmax = 0
    ymax = 0
    xmin = appr_s[0][0]
    ymin = appr_s[0][1]
    for d in appr_s:
        if d[0] > xmax:
            xmax = d[0]
        if d[1] > ymax:
            ymax = d[1]
        if d[0] < xmin:
            xmin = d[0]
        if d[1] < ymin:
            ymin = d[1]
    for i in range(0, len(appr_s)):
        appr_s[i][0] -= xmin
        appr_s[i][0] = appr_s[i][0] / xmax
        appr_s[i][1] -= ymin
        appr_s[i][1] = appr_s[i][1] / ymax
    
    # fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(9, 4))
    # ax2.plot(contour[:, 0], contour[:, 1])
    # ax1.plot(appr_s[:, 0], appr_s[:, 1])
    # plt.show()

    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    global llc_p, llc_q
    global cur_pose
    
    gripper.open()

    marker_pose = Pose()
    marker_pose.position = marker_p
    marker_pose.orientation = marker_q
    llc_pose = Pose()
    llc_pose.position = llc_p
    llc_pose.orientation = llc_q
    cur_pose = Pose()
    cur_pose.position = llc_p
    cur_pose.orientation = llc_q

    move_to(marker_pose, 0.3, 5)
    rospy.sleep(2.0)
    gripper.close()
    rospy.sleep(1.0)
    marker_pose.position.z += 0.1
    move_to(marker_pose, 0.3, 5)
    llc_pose.position.z += 0.1
    llc_pose.position.y += 0.1
    move_to(llc_pose, 0.3, 5)

    cur_pose.position.x += appr_s[0][0] * board_width
    cur_pose.position.y -= appr_s[0][1] * board_height
    move_to(cur_pose, 0.4, 5)

    # EDIT THIS IF BOB NEEDS TO DRAW LOWER
    cur_pose.position.z -= 0.048

    move_to(cur_pose, 0.015, 10)
    for i in range(1, len(appr_s)):
        x_diff = (appr_s[i][0] * board_width) - (appr_s[i - 1][0] * board_width)
        y_diff = (appr_s[i][1] * board_height) - (appr_s[i - 1][1] * board_width)
        cur_pose.position.x += (appr_s[i][0] * board_width) - (appr_s[i - 1][0] * board_width)
        cur_pose.position.y -= (appr_s[i][1] * board_height) - (appr_s[i - 1][1] * board_width)
        move_to(cur_pose, 0.02, 10)

def rotate(deg):
    global g_limb
    global cur_pose, subscriber_pose_state

    deg = deg % (2 * math.pi)
    old_angles = g_limb.joint_angles()
    old_angles['right_j6'] = (deg)
    g_limb.move_to_joint_positions(old_angles, 10)
    cur_pose.orientation = subscriber_pose_state.orientation


def main():
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    global subscriber_pose_state
    init()
    g_limb.move_to_neutral()
    gripper.open()
    #draw_image("tree.jpeg")
    draw_circle_lines()
    g_limb.move_to_neutral()
    
    marker_pose = Pose()
    marker_pose.orientation = marker_q
    marker_pose.position = marker_p
    move_to(marker_pose, 0.2, 5)
    gripper.open()
    g_limb.move_to_neutral()


if __name__ == "__main__":
    main()


#Information prints
#rospy.loginfo("Old Hand Pose:\n %s" % str(g_limb._tip_states.states[0].pose))
#rospy.loginfo("Old Joint Angles:\n %s" % str(g_limb.joint_angles()))
