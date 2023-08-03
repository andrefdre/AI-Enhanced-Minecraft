#!/usr/bin/python3

import time
from functools import partial
import requests
from ai_enhanced_minecraft_messages.msg import KeyStrokes

# ROS2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class KeyCaptureNode(Node):
    
    def __init__(self):
        super().__init__('key_capture')
        self.publisher = self.create_publisher(String, 'keys_pressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        try:    
            response = requests.get('http://localhost:8080/player_actions')
            print(response.content)
        except:
            # print("Error connecting to server")
            print(f"Received an error response: {response.status_code} - {response.text}")

        read_strokes(response)

def read_strokes(response):
    
    # content_str = response.text
    # # Split the content into lines
    # lines = content_str.split('\n')
    # # Initialize a dictionary to store the key-value pairs
    # data_dict = {}

    # # Iterate through each line and split it into key-value pairs
    # for line in lines:
    #     if line.strip():  # Skip empty lines
    #         key, value = line.split(': ')
    #         data_dict[key.strip()] = value.strip()
    
    # print(data_dict)
    # print('\n\n')
    values = KeyStrokes()
    # values.a = response.json()['a']
    # print(values.a)
    # self.publisher.publish(msg)

def main():

    rclpy.init()
    node = KeyCaptureNode()
    
    rclpy.spin(node)    

    
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()