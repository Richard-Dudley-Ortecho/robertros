# Sample program to solve for the Inverse Kinematics of a particular hand position and send the robot arm there

import rospy
import intera_interface
from sawyer_pykdl import sawyer_kinematics


rospy.init_node("test_ik")

kin = sawyer_kinematics('right')
limb = intera_interface.Limb('right')
kinematics = sawyer_kinematics('right')
limb.move_to_neutral()
xyz_pos = [0.44960210426, 0.161201409345, 0.215379976695]

coor = [ 0.82247637,  0.11408647,  0.26880369,  0.74201184,  0.66959782,
       -0.00727163, -0.03169108]


# print("THIS: ", kinematics.forward_position_kinematics())
x = dict(zip(limb._joint_names, kinematics.inverse_kinematics(coor)))
limb.move_to_joint_positions(x)
