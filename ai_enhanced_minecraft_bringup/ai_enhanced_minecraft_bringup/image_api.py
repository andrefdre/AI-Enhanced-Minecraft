#!/usr/bin/python3

from functools import partial
import requests
import cv2
import numpy as np

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class Image_API(Node):
    def __init__(self):
        super().__init__('image_api')
        self.publisher_ = self.create_publisher(Image, 'minecraft_frame', 10)
        self.timer = self.create_timer(0, self.timer_callback)
        self.image = None
        self.bridge = CvBridge()

    def timer_callback(self):
        try:
            response = requests.get('http://localhost:8070/image')
            self.image = response.content
        except:
            print(f"Received an error response: {response.status_code} - {response.text}")

        image_array = cv2.imdecode(np.frombuffer(self.image, np.uint8), cv2.IMREAD_COLOR)
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)

        self.publisher_.publish(self.bridge.cv2_to_imgmsg(image_array))

def main():
    rclpy.init()
    node = Image_API() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()