#!/usr/bin/python3

from functools import partial
import requests

# ROS2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class AI_API(Node):
    def __init__(self):
        super().__init__('minecraft_api')
        self.publisher = self.create_publisher(String, 'pose_api', 10)
        self.timer = self.create_timer(0, self.timer_callback)

    def timer_callback(self):
        try:
            response = requests.get('http://localhost:8080/player')
            print(response.text)
            msg = String()
            msg.data = response.text
            self.publisher.publish(msg)
        except:
            print("Error connecting to server")

def main():
    rclpy.init()
    node = AI_API()

    rclpy.spin(node)    

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()