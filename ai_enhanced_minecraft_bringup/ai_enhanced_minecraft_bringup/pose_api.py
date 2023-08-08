#!/usr/bin/python3

from functools import partial
import requests

# ROS2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ai_enhanced_minecraft_messages.msg import Pose


class Pose_API(Node):
    def __init__(self):
        super().__init__('pose_api')
        self.publisher = self.create_publisher(Pose, 'pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            response = requests.get('http://localhost:8070/player_positions')
        except:
            print(f"Received an error response: {response.status_code} - {response.text}")

        msg = Pose()
        msg.x = response.json()['x']
        msg.y = response.json()['y']
        msg.z = response.json()['z']
        msg.yaw = response.json()['yaw']
        msg.pitch = response.json()['pitch']
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = Pose_API()

    rclpy.spin(node)    

    node.destroy_node()
    
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()