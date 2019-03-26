#!/usr/bin/env python
"""
Example for commanding joints
"""

import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
pos_3d_cam = []
pos_3d_world = []
bridge = CvBridge()

def plan_trajectory(waypoints, num):
    traj = np.zeros(5)
    finalTraj = [traj]
    current_step = np.zeros(5)
    for next_step in waypoints:
        step_size = (next_step - current_step)/num
        for j in range(num):
            current_step = current_step + step_size
            # traj = np.concatenate((traj, current_step))
            finalTraj.append(current_step)
    finalTraj = np.stack(finalTraj)
    # print(finalTraj.shape)
    return finalTraj

def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)


def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)


def get_artag_pos_in_world(joint):
    # fk from base to artag
    return [0.0, 0.0, 0.0]
    

def sub_callback(markers):
    global pos_3d_cam
    for marker in markers:
        pos = marker.pose.position
        # ori = marker.pose.orientation
        pos_3d_cam.append([pos.x, pos.y, pos.z])

def img_callback(Imgs):
    cv_image = bridge.imgmsg_to_cv2(Imgs,"bgr8")
    cv2.imshow("Image",cv_image)
    cv2.waitKey(3)
    # cv2.destroyAllWindows()
    # print(np.int8(np.array(Imgs.data))) 

def main():
    global pos_3d_cam
    rospy.init_node('command_joints_example', anonymous=True)

    intrinsic_matrix = [ 
        [614.357421875,  0.0,            310.2319641113281],
        [0.0,            614.494140625,  244.32691955566406], 
        [0.0,            0.0,            1.0]
    ]

    target_joints = [
        [0.408, 0.721, -0.471, -1.4, 0.920],
        [-0.675, 0, 0.23, 1, -0.70],
        [0,0,0,0,0]
    ]
    target_joints = plan_trajectory(target_joints,50)
    # pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
    #                       JointState, queue_size=1)
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, sub_callback)
    sub_img = rospy.Subscriber("/camera/color/image_raw", Image, img_callback)
    # rospy.sleep(2)
    # home_arm(pub)

    for joint in target_joints:
    #   set_arm_joint(pub, joint)
      # print(joint)
    #   rospy.sleep(0.2)
      pos_3d_world.append(get_artag_pos_in_world(joint))

    # convert pos_3d_cam to 2d pos in image
    intrinsic_matrix = np.array(intrinsic_matrix)
    pos_3d_cam = np.array(pos_3d_cam)
    # pos_zs = pos_3d_cam[:,2]
    pos_2d =  intrinsic_matrix.dot(pos_3d_cam)
    us = pos_2d[0,:]/pos_2d[2,:]
    vs = pos_2d[1,:]/pos_2d[2,:]

    

    print("+++++++++++", pos_2d[0], pos_2d[1])
    print(pos_3d_cam[0], pos_3d_cam[1])

    rospy.spin()
    

    # home_arm(pub)




if __name__ == "__main__":
    main()