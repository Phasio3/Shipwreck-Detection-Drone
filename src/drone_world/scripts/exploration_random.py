#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import random
import time

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')
        
        self.x = 0.0
        self.y = 0.0


        # Publisher vers le drone
        self.pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.pose_pub = self.create_publisher(Pose, "/drone/pose", 10)
	
        # Timer : génère une nouvelle commande toutes les 1 secondes
        self.timer = self.create_timer(1.0, self.move_random)

        self.get_logger().info("Exploration aléatoire démarrée !")

    def move_random(self):
        msg = Twist()

        # Vitesse linéaire aléatoire en X et Y
        msg.linear.x = random.uniform(-1.0, 1.0)
        msg.linear.y = random.uniform(-1.0, 1.0)

        # Rotation aléatoire (optionnel)
        msg.angular.z = random.uniform(-1.0, 1.0)
        
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        self.pose_pub.publish(pose)
	
        self.pub.publish(msg)
        self.get_logger().info(f"cmd_vel envoyée : x={msg.linear.x:.2f}, y={msg.linear.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
