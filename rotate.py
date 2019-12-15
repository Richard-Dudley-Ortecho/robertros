# havent tried it on the real thing, doesnt work in simulator without making the entire robot
# collapse in a heap. test with care

def rotate():
    global g_limb
    old_angles = g_limb.joint_angles()
    print(old_angles)
    old_angles['right_j6'] = (1.707)
    print(old_angles)
    g_limb.move_to_joint_positions(old_angles)
