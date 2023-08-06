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
    
    def timer_callback(self):
        try:    
            response = requests.get('http://localhost:8070/player_inv')
        except:
            print(f"Received an error response: {response.status_code} - {response.text}")

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


def main():

    rclpy.init()
    node = GetInventory()
    
    rclpy.spin(node)    

    
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()