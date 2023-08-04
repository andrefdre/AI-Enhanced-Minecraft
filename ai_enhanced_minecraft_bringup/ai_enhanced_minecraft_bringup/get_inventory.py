#!/usr/bin/python3
import json
import time
from functools import partial
import requests

# ROS2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class GetInventory(Node):
    
    def __init__(self):
        super().__init__('get_inventory')
        self.publisher = self.create_publisher(String, 'inventory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        try:    
            response = requests.get('http://localhost:8070/player_inv')
            # print(response.content)
        except:
            # print("Error connecting to server")
            print(f"Received an error response: {response.status_code} - {response.text}")

        read_strokes(response)

def read_strokes(response):
    
    data = response.json()

    print(data)


def main():

    rclpy.init()
    node = GetInventory()
    
    rclpy.spin(node)    

    
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()