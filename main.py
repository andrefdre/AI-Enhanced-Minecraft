
import cv2 as cv
import numpy as np
import os
from time import time
from windowcapture import WindowCapture
from objects_id import Vision
import glob

# Get a list of the windows names
# windows_list = WindowCapture.list_window_names()
                         
# window to be captured
window = 'Minecraft* 1.19.2 - Multiplayer (3rd-party Server)'

# if window not in windows_list:
#     print(f'The "{window}" window is not currently open.')
#     print('Closing the screen streamer app...')

# else:

# initialize the WindowCapture class
wincap = WindowCapture(window)
inventory_bar = Vision('./Imgs/Inventory1.jpg')

loop_time = time()

while(True):
    
    # get an updated image of the game
    screenshot = wincap.get_screenshot()

    # cv.imshow('Computer Vision', screenshot)

    points = inventory_bar.find(screenshot, 0.45, 'rectangles')

    # debug the loop rate
    print('FPS {}'.format(1 / (time() - loop_time)))
    loop_time = time()

    # press 'q' with the output window focused to exit.
    # waits 1 ms every loop to process key presses
    if cv.waitKey(1) == ord('q'):
        cv.destroyAllWindows()
        break

print('Done.')
