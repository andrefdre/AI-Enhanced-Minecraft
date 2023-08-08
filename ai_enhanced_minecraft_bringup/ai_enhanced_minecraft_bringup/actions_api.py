#!/usr/bin/python3

from functools import partial
import requests

# ROS2 imports
import rclpy
from rclpy.node import Node

from ai_enhanced_minecraft_messages.msg import Actions


class Actions_API(Node):
    def __init__(self):
        super().__init__('actions_api')
        self.publisher = self.create_publisher(Actions, 'actions', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            response = requests.get('http://localhost:8070/player_actions')
        except:
            print(f"Received an error response: {response.status_code} - {response.text}")

        msg = Actions()
        msg.right_click = response.json()['Right_click']
        msg.left_click = response.json()['Left_click']
        msg.jump_key = response.json()['Jump_key']
        msg.crouch_key = response.json()['Crouch_key']
        msg.sprint_key = response.json()['Sprint_key']
        msg.w_key = response.json()['W_key']
        msg.a_key = response.json()['A_key']
        msg.s_key = response.json()['S_key']
        msg.d_key = response.json()['D_key']
        msg.mouse_y_position = response.json()['Mouse_Y_position']
        msg.mouse_x_position = response.json()['Mouse_X_position']

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = Actions_API()

    rclpy.spin(node)    

    node.destroy_node()
    
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()