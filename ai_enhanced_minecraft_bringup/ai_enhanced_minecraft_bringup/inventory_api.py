#!/usr/bin/python3
import json
import time
from functools import partial
import requests
from ai_enhanced_minecraft_messages.msg import Inventory, Slot

# ROS2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class GetInventory(Node):
    
    def __init__(self):
        super().__init__('get_inventory')
        self.publisher = self.create_publisher(Slot, 'inventory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.connected_once = False
    
    def timer_callback(self):
        
        # Just to debug the initial connection to the server
        while self.connected_once == False:    
            try:
                response = requests.get('http://localhost:8070/player_inv')
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
                response = requests.get('http://localhost:8070/player_inv')
                response.raise_for_status()
                
                # New data received in json
                data = response.json()

                # Instantiate the Slot message type
                msg = Slot()

                for i in range(len(data)):
                    msg_2 = Inventory()
                    msg_2.item = data[f'Slot {i}']['item']
                    msg_2.amount = data[f'Slot {i}']['amount']
                    
                    msg.slots[i] = msg_2

                self.publisher.publish(msg)

            except:
                print("\n Lost connection to server... \n")
                lost_connection = True
                

        # If we lost connection and need to try and reconnect
        while lost_connection == True:  
            try:
                response = requests.get('http://localhost:8070/player_inv')
                response.raise_for_status()
                lost_connection = False
                print("Connection successfully re-established!")
                break
            except:
                print("Trying to re-establish connection to the server...")
                time.sleep(1)


def main():

    rclpy.init()
    node = GetInventory()
    
    rclpy.spin(node)    

    
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()