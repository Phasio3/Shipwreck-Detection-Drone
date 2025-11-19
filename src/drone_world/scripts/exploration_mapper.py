#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

import math
import numpy as np


class ExplorationMapper(Node):

    def __init__(self):
        super().__init__("exploration_mapper")

        # Size of the map
        self.width = 400
        self.height = 400
        self.resolution = 0.1  # 10 cm / cell

        # Internal numpy map (-1 = unknown)
        self.map_array = np.full((self.height, self.width), -1, dtype=np.int8)

        # Create message
        self.map_msg = OccupancyGrid()
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.origin.position.x = - (self.width * self.resolution) / 2
        self.map_msg.info.origin.position.y = - (self.height * self.resolution) / 2

        # Publishers / subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, "/exploration_map", 10)
        self.pose_sub = self.create_subscription(
            Pose, "/drone/pose", self.pose_callback, 10
        )

        # Publish map regularly
        self.create_timer(0.2, self.publish_map)

        self.get_logger().info("Mapper started !")

    def pose_callback(self, pose: Pose):
        """Mark a white circle around the drone position."""

        x = pose.position.x
        y = pose.position.y

        # Convert to map coordinates
        mx = int((x - self.map_msg.info.origin.position.x) / self.resolution)
        my = int((y - self.map_msg.info.origin.position.y) / self.resolution)

        radius = 8  # cells

        for iy in range(my - radius, my + radius):
            for ix in range(mx - radius, mx + radius):
                if 0 <= ix < self.width and 0 <= iy < self.height:
                    if (ix - mx)**2 + (iy - my)**2 <= radius**2:
                        self.map_array[iy, ix] = 0  # white = free space

    def publish_map(self):
        self.map_msg.data = self.map_array.flatten().tolist()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

