#!/usr/bin/python3

import time
from pynput import keyboard
from functools import partial

# ROS2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class KeyCaptureNode(Node):
    def __init__(self):
        super().__init__('key_capture')
        self.publisher = self.create_publisher(String, 'keys_pressed', 10)
        self.on_press_partial = partial(self.on_press_callback, publisher=self.publisher)

    def on_press_callback(self,key, publisher):
        msg = String()
        msg.data = str(key)
        publisher.publish(msg)

def main():
    rclpy.init()
    node = KeyCaptureNode()

    print("Press any key to start capturing keys...")
    while True:
        listener = keyboard.Listener(on_press=node.on_press_partial)
        listener.start()
        rclpy.spin(node)


if __name__ == '__main__':
    main()