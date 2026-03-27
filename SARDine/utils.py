import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray


def PoseStamped_dist(pose1, pose2):
    pose1_np = np.array([
        pose1.pose.position.x,
        pose1.pose.position.y,
        pose1.pose.position.z,
    ])

    pose2_np = np.array([
        pose2.pose.position.x,
        pose2.pose.position.y,
        pose2.pose.position.z,
    ])

    return np.linalg.norm(pose1_np - pose2_np)

def NewPoseStamped(
        stamp,
        pos_x,
        pos_y,
        pos_z,
        orient_x,
        orient_y,
        orient_z,
        orient_w
):
    new_pose = PoseStamped()
    new_pose.header.stamp = stamp
    new_pose.header.frame_id = "map"

    new_pose.pose.position.x = pos_x
    new_pose.pose.position.y = pos_y
    new_pose.pose.position.z = pos_z

    new_pose.pose.orientation.x = orient_x
    new_pose.pose.orientation.y = orient_y
    new_pose.pose.orientation.z = orient_z
    new_pose.pose.orientation.w = orient_w
    
    return new_pose

