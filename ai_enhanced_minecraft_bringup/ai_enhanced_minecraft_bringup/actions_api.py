#!/usr/bin/python3

from functools import partial
import requests
import time

# ROS2 imports
import rclpy
from rclpy.node import Node

from ai_enhanced_minecraft_messages.msg import Actions


class Actions_API(Node):
    def __init__(self):
        super().__init__('actions_api')
        self.publisher = self.create_publisher(Actions, 'actions', 10)
        self.timer = self.create_timer(0, self.timer_callback)
        self.connected_once = False

    def timer_callback(self):
        
        # Just to debug the initial connection to the server
        while self.connected_once == False:    
            try:
                response = requests.get('http://localhost:8070/player_actions')
                response.raise_for_status()
                print("Connection successfully established!")
                self.connected_once = True
                lost_connection = False
                break
            except:
                print("Cannot establish initial connection to the server. Retrying in 1 second.")
                time.sleep(1)
        
        while lost_connection == True:  
            try:
                response = requests.get('http://localhost:8070/player_actions')
                response.raise_for_status()
                lost_connection = False
                print("Connection successfully re-established!")
                break
            except:
                print("Trying to re-establish connection to the server...")
                time.sleep(1)


        while lost_connection == False:
            
            try:
                response = requests.get('http://localhost:8070/player_actions')
                response.raise_for_status()
                # building the message
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
                
                # publish message to topic
                self.publisher.publish(msg)

            except:
                print("\n Lost connection to server... \n")
                lost_connection = True


def main():
    rclpy.init()
    node = Actions_API()

    rclpy.spin(node)    

    node.destroy_node()
    
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()