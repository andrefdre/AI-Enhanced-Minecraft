#!/usr/bin/env python3

"""
    Script for writing a dataset for training a Deep-learning model.
    The image is obtained from the topic specified in the parameter 'image_raw_topic'.
    The steering angle is obtained from the topic specified in the parameter 'twist_cmd_topic'.
    The dataset is saved in the default path to the Automec folder set in the Environment variable.
"""

# Imports
import os
import pathlib
import shutil
from datetime import datetime
from functools import partial
from typing import Any
import cv2
import pandas as pd
import yaml
from PIL import Image as Image_pil

# ROS2 imports
import rclpy
from rclpy.node import Node
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image # Image is the message type
from ai_enhanced_minecraft_messages.msg import Actions

class Generate_Dataset(Node):
    def __init__(self):
        super().__init__('generate_dataset')
          # Retrieving parameters
        image_topic = self.declare_parameter('image_topic' , 'minecraft_frame').value
        actions_topic = self.declare_parameter('actions_topic' , 'actions').value
        self.image_width = self.declare_parameter('image_width' , 640).value
        self.image_height = self.declare_parameter('image_height' , 480).value

        self.publisher = self.create_subscription(Image, image_topic , self.imgRgbCallback, 10)
        self.publisher = self.create_subscription(Actions, actions_topic , self.actionsMsgCallback, 10)
        self.timer = self.create_timer(0, self.loop_callback)

        minecraft_path = os.environ.get('Minecraft_Path')
        self.data_path = f'{minecraft_path}/datasets/' + "-" + datetime.now().strftime("%d-%m-%Hh%Mm%Ss")

        # If the path does not exist, create it
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)
            data_path_imgs = self.data_path + '/IMG'
            os.makedirs(data_path_imgs)
        else:
            print('Folder already exists, please try again with a different folder!')
            exit(1)

        self.info_data = dict(
            dataset=dict(
                developer=os.getlogin(),
                image_number=0,
                image_width=self.image_width,
                image_height=self.image_height,
            )
        )

        print(self.data_path)

        self.win_name = 'Minecraft View'
        cv2.namedWindow(winname=self.win_name,flags=cv2.WINDOW_NORMAL)

        # Create an object of the CvBridge class
        self.bridge = CvBridge()
        self.begin_img = False
        self.begin_actions = False
        self.counter = 0

        # Create pandas dataframe
        self.dataset_log = pd.DataFrame(
            columns=['Image', 'right_click', 'left_click', 'jump_key', 'crouch_key', 'sprint_key', 'w', 'a', 's', 'd', 'mouse_x', 'mouse_y'])

    def loop_callback(self):

        if not self.begin_img:
            return
        
        if not self.begin_actions:
            return

        cv2.imshow(self.win_name, self.img_rgb)
        key = cv2.waitKey(1)
        
        # save on shutdown...
        if key == ord('s'):  
            self.save_dataset(self.info_data, self.data_path, self.dataset_log)
            exit(0)

        if key == ord('q'):
            confirmation = input("\n\nYou have pressed q[uit]: are you sure you want to close"
                                " WITHOUT saving the dataset? (type 'yes' TO DISCARD the dataset,"
                                " type 'no' or 'save' to SAVE the dataset): ")
            if confirmation == "yes":
                shutil.rmtree(self.data_path)
                print("All done, exiting ROS...")
            elif confirmation in {'no', 'save'}:
                self.save_dataset(self.info_data, self.data_path, self.dataset_log)
                exit(0)

        curr_time = datetime.now()
        image_name = f'{str(curr_time.year)}_{str(curr_time.month)}_{str(curr_time.day)}_' \
                     f'_{str(curr_time.hour)}_{str(curr_time.minute)}_{str(curr_time.second)}_' \
                     f'_{str(curr_time.microsecond)}.jpg'

        # add image, angle and velocity to the driving_log pandas
        row = pd.DataFrame(data=[[image_name, self.actions.right_click, self.actions.left_click, self.actions.jump_key, self.actions.crouch_key, 
                            self.actions.sprint_key, self.actions.w_key, self.actions.a_key, self.actions.s_key, self.actions.d_key, 
                            self.actions.mouse_x_position, self.actions.mouse_y_position]],
                           columns=['Image', 'right_click', 'left_click', 'jump_key', 'crouch_key', 'sprint_key', 'w', 'a', 's', 'd', 'mouse_x', 'mouse_y'])
        self.dataset_log = pd.concat([self.dataset_log, row])

        # save image
        dim = (self.image_width, self.image_height)
        img_rgb_resize = cv2.resize(self.img_rgb, dim, interpolation=cv2.INTER_AREA)
        image_saved = Image_pil.fromarray(img_rgb_resize)
        image_saved.save(self.data_path + '/IMG/' + image_name)
        self.counter += 1
        print(f'Image Saved: {self.counter}', end="\r")


    # Callback Function to receive the cmd values
    def actionsMsgCallback(self , message):
        self.actions = message

        self.begin_actions = True
       

    # Callback function to receive image
    def imgRgbCallback(self , message):
        """Callback for receiving the image from the car.
            Args:
                message (Image): ROS Image message.
                config (dict): Dictionary with the configuration.
        """
        self.img_rgb = self.bridge.imgmsg_to_cv2(message, "passthrough")

        self.begin_img = True


    def save_dataset(self , info_data, data_path, dataset_log):
        """Saves dataset on a .csv file and a metadata file in YAML

        Args:
            date (string): starting date of dataset
            info_data (dict): dictionary with metadata
            data_path (string): dataset path
            dataset_log (pandas dataset): dataset with actions and image info
        """
        print('EXITING...')
        dataset_log.to_csv(data_path + '/dataset_log.csv', mode='a', index=False, header=False)
        info_data['dataset']['image_number'] = len(list(os.listdir(data_path + "/IMG/")))

        info_data['dataset']['date'] = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open(data_path + '/info.yaml', 'w') as outfile:
            yaml.dump(info_data, outfile, default_flow_style=False)


def main():
    # Init Node
    rclpy.init()
    node = Generate_Dataset()

    rclpy.spin(node)    

    node.destroy_node()
    
    rclpy.shutdown()

   
if __name__ == '__main__':
    main()