import rospy
import intera_interface
from sawyer_pykdl import sawyer_kinematics


rospy.init_node("test_ik")
kinematics = sawyer_kinematics('right')
kinematics.forward_position_kinematics()