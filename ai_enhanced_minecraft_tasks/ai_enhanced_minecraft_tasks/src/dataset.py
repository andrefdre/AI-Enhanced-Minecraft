#!/usr/bin/python3

import numpy as np
import torch
from PIL import Image
from torchvision import transforms

class Dataset(torch.utils.data.Dataset):
    def __init__(self,image_filenames):
        #super().init()
        self.image_filenames = image_filenames
        self.num_images= len(self.image_filenames)

        self.labels_string=[]

        for image_filename in self.image_filenames:
            label = self.getClassFromFilename(image_filename)
            self.labels_string.append(label)

        # Gets the list of the items
        self.classes_list = GetClassListFromFolder()

        # Creates the labels comparing the string of the item with the folder structure
        self.labels=[]
        for label in self.labels_string:
            label_idx=self.classes_list.index(label)
            self.labels.append(label_idx)

        # Create a set of transformations
        self.transforms = transforms.Compose([
            #transforms.Resize((64,64)),
            transforms.ToTensor()
        ])
    
    # Function that will get the class from the file structure
    def getClassFromFilename(self, filename):
        parts = filename.split('/')
        # Get the third item counting from the end
        part = parts[len(parts)-3]
        return part

   


    def __getitem__(self,index): # returns a specific x,y of the datasets
        # Get the image
        image_pill = Image.open(self.image_filenames[index])
        # Resize the image without changing the aspect ratio
        image_pill.thumbnail((64,64))
        # Convert the image to a numpy array
        image_array = np.asarray(image_pill)
        # Pad the image to 64x64
        image_padded=self.expand2square(image_array)
        # Convert the image to a tensor
        image_t= self.transforms(np.array(image_padded))
        # Get the label
        label = self.labels[index]
        return image_t , label
       
    def __len__(self):
        return self.num_images