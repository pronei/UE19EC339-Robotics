#!/usr/bin/env python3

import rospy

from arm_control import arm_control
from inverse_kinematics import iKine
from spawner import model_spawner

if __name__ == "__main__":
    rospy.init_node("arm_control", anonymous=True)

    arm = arm_control()
    arm.reset_arm()

    model_spawner = model_spawner()
    for index in range(1, 6):
        model_spawner.spawn_contact_model("box" + str(index))

    points = model_spawner.get_points_to_reach()
    print("The points to reach are:", points)

    for point in points:
        theta, vert_dist, horz_dist = iKine(point)
        print('Point =', point)
        print('theta =', theta)
        print('vert_dist =',vert_dist)
        print('hor_dist =', horz_dist)
        arm.send_data_to_arm(theta, vert_dist, horz_dist)
        arm.close_gripper()
        arm.open_gripper()
