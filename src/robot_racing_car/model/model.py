#!/usr/bin/env python

import csv
import cv2
import numpy as np

csv_lines = []
with open("../training_data/training_data.csv") as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for line in reader:
        csv_lines.append(line)

images = []
steering_commands = []

image_width = 320
image_height = 240
scale = 4

for csv_line in csv_lines:
    source_path = '../training_data/' + csv_line[0]
    image = cv2.imread(source_path)
    
    smallImg = np.empty((image.shape[1]/scale, image.shape[0]/scale))
    images.append(cv2.resize(image, smallImg.shape, interpolation=cv2.INTER_LINEAR))
    steering_commands.append(float(csv_line[1]))
    
    # TODO: Figure out how to generate new data based on existing
    #images.append(cv2.flip(image, 1))
    #steering_commands.append(float(line[3]) * -1.0)
    
X_train = np.array(images)
y_train = np.array(steering_commands)

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Activation, Convolution2D, Cropping2D

def get_model():
    model = Sequential()

    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape = (image_height, image_width, 3)))
    model.add(Cropping2D(cropping=((106,0), (0,0)), input_shape=(image_height, image_width, 3)))
    
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2)))
    model.add(Activation('relu'))
    
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2)))
    model.add(Activation('relu'))
    
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2)))
    model.add(Activation('relu'))

    model.add(Convolution2D(64, 3, 3, subsample=(1, 1)))
    model.add(Activation('relu'))

    model.add(Convolution2D(64, 3, 3, subsample=(1, 1))) 
    model.add(Activation('relu'))

    model.add(Flatten())    

    model.add(Dense(100))
    model.add(Activation('relu'))
    
    model.add(Dense(50))
    model.add(Activation('relu'))
    
    model.add(Dense(10))
    model.add(Activation('relu'))

    model.add(Dense(1))

    model.compile(optimizer="adam", loss="mse", metrics=['binary_accuracy']) 
    
    return model

model = get_model()

model.fit(X_train, y_train, validation_split = 0.2, shuffle = True, nb_epoch = 10)

model.save("./model.h5")  

    

