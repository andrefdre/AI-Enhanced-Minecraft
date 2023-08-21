#!/usr/bin/python3

from functools import partial
import requests
import time 

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
        self.connected_once = False

    def timer_callback(self):
        
        # Just to debug the initial connection to the server
        while self.connected_once == False:    
            
            try:
                response = requests.get('http://localhost:8070/player_positions')
                response.raise_for_status()
                print('Connection successfully established!')
                self.connected_once = True
                lost_connection = False
                break
            except:
                print("Cannot establish initial connection to the server. Retrying in 1 second.")
                time.sleep(1)

        # While we have connection to the server
        while lost_connection == False:
            
            try:

                response = requests.get('http://localhost:8070/player_positions')
                response.raise_for_status()
               
                # building the message
                msg = Pose()
                msg.x = response.json()['x']
                msg.y = response.json()['y']
                msg.z = response.json()['z']
                msg.yaw = response.json()['yaw']
                msg.pitch = response.json()['pitch']
                
                # publish message to topic
                self.publisher.publish(msg)

            except:
                print("\n Lost connection to server... \n")
                lost_connection = True

        # If we lost connection and need to try and reconnect
        while lost_connection == True:  
            try:
                response = requests.get('http://localhost:8070/player_positions')
                response.raise_for_status()
                lost_connection = False
                print("Connection successfully re-established!")
                break
            except:
                print("Trying to re-establish connection to the server...")
                time.sleep(1)


def main():
    rclpy.init()
    node = Pose_API()

    rclpy.spin(node)    

    node.destroy_node()
    
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()