#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.map_pub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.nav_client = ActionClient(
            self,
            'NavigationToPose',
            'navigation_to_pose'
        )