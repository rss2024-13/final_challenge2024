import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseArray, Point
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose, Quaternion

from .utils import LineTrajectory
from typing import List, Tuple

import math
import numpy as np


class MotorControl(Node):
    """ Implements motor control by using path planning and line following. The path planning 
    drive topic is /path_planner_drive and the line following drive topic is /line_follower_drive.

    We use the following logic to determine which of the path planner and line follower to tune into:

        1) if we cannot identify the line, we use the path planner
        2) if we are a certain distance away from the line, we use the path planner
        3) otherwise, we use the line follower

    We decided this was best because we are going to make u-turns at corners (no line) if needed and 
    following the road is more important than going after the goal when we are far away.

    We still need to include the stop sign controls and stop light controls so that we can include 
    that logic as well.
    """

    def __init__(self):
        super().__init__("motor_control")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.path_planner_drive = '/path_planner_drive'
        self.line_follower_drive = '/line_follower_drive'

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )


    def pose_callback(self, odometry_msg):
        
        drive_cmd = None  # we want to determine which drive command to use (path planner vs line follower)
        self.drive_pub.publish(drive_cmd)





def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControl()
    rclpy.spin(motor_control)
    rclpy.shutdown()
