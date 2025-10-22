#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time

def create_pose_stamped(navigator, x, y, theta):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, theta)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Initial pose in the map frame
    initial_pose = create_pose_stamped(nav, 0.0, -0.2, 0.0)
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    # Send Nav2 Goals
    waypoints = [
        create_pose_stamped(nav, 5.0, 1.0, math.radians(90)),
        create_pose_stamped(nav, 5.0, 4.5, math.radians(0)),
        create_pose_stamped(nav, 8.0, 4.5, math.radians(270)),
        create_pose_stamped(nav, 8.0, 1.0, math.radians(180)),
    ]
    while rclpy.ok():
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete() and rclpy.ok():
            time.sleep(0.1)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
