#!/usr/bin/python3
import json
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
        self.publisher = self.create_publisher(KeyStrokes, 'keys_pressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        try:    
            response = requests.get('http://localhost:8070/player_actions')
            # print(response.content)
        except:
            # print("Error connecting to server")
            print(f"Received an error response: {response.status_code} - {response.text}")

        msg = read_strokes(response)
        
        self.publisher.publish(msg)


def read_strokes(response):
    
    data = response.json()
    values = KeyStrokes()
    
    values.w = data['key_w']
    values.a = data['key_a']
    values.s = data['key_s']
    values.d = data['key_d']
    values.jump = data['key_jump']
    values.sprint = data['key_sprint']
    values.crouch = data['key_crouch']
    values.left_mouse = data['mouse_left']
    values.right_mouse = data['mouse_right']
    values.mouse_x = data['mouse_x_position']
    values.mouse_y = data['mouse_y_position']

    return values


def main():

    rclpy.init()
    node = KeyCaptureNode()
    
    rclpy.spin(node)    

    
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()