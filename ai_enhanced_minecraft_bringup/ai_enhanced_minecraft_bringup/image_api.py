#!/usr/bin/python3

from functools import partial
import requests
import cv2
import numpy as np
import time
# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class Image_API(Node):
    def __init__(self):
        super().__init__('image_api')
        self.publisher_ = self.create_publisher(Image, 'minecraft_frame', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.image = None
        self.bridge = CvBridge()
        self.connected_once = False


        
    def timer_callback(self):
        
        # Just to debug the initial connection to the server
        while self.connected_once == False:    
            
            try:
                response = requests.get('http://localhost:8070/image')
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
                response = requests.get('http://localhost:8070/image')
                response.raise_for_status()

                self.image = response.content
                
                image_array = cv2.imdecode(np.frombuffer(self.image, np.uint8), cv2.IMREAD_COLOR)
                image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)

                self.publisher_.publish(self.bridge.cv2_to_imgmsg(image_array))
                cv2.imshow("image", image_array)
            
                cv2.waitKey(1)

            except:
                print("\n Lost connection to server... \n")
                lost_connection = True

        
        # If we lost connection and need to try and reconnect
        while lost_connection == True:  
            try:
                response = requests.get('http://localhost:8070/image')
                response.raise_for_status()
                lost_connection = False
                print("Connection successfully re-established!")
                break
            except:
                print("Trying to re-establish connection to the server...")
                time.sleep(1)

def main():
    rclpy.init()
    node = Image_API() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()