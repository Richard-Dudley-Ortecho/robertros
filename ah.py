# from pyrobot import Robot
# import numpy as np
# import time
# import rospy

# import math
# import intera_interface

# from sawyer_pykdl import sawyer_kinematics

# rospy.init_node("ah")

# # robot = Robot('sawyer',
# #               use_base=False,
# #               use_camera=False)



from pyrobot import Robot
import numpy as np
import time
import math

robot = Robot('sawyer',
              use_base=False,
              use_camera=False)

robot.arm.go_home()
target_joint = [0.704, -0.455, -0.159, 1.395, -1.240, 1.069, 2.477]
robot.arm.set_joint_positions(target_joint, plan=False)
robot.arm.go_home()
# limb = intera_interface.Limb('right')

# plan = False
# limb.move_to_neutral()
# time.sleep(1)
# displacement = np.array([0.15, 0, 0])
# limb.move_ee_xyz(displacement, plan=plan)
# time.sleep(1)
# displacement = np.array([0., 0.15, 0])
# limb.move_ee_xyz(displacement, plan=plan)
# time.sleep(1)
# displacement = np.array([0., 0., 0.15])
# limb.move_ee_xyz(displacement, plan=plan)
# time.sleep(1)
# limb.go_home()