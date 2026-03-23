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

