#!/usr/bin/env python

import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

from keras.models import Sequential
from keras import optimizers
from keras.models import Model
from keras.utils import to_categorical
from keras.layers import Flatten, Dense, Lambda, Input, Activation, Convolution2D, Cropping2D, Dropout

import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.optimizers import Adam, RMSprop, SGD
from keras.callbacks import ModelCheckpoint

image_width = 320
image_height = 240

def linear_bin(value_to_bin):
    value_to_bin = value_to_bin + 1
    binned_value = round(value_to_bin / (2.0 / 14.0))

    return int(binned_value)

def bin_matrix(matrix_to_bin):
    binned_matrix = []
    for value_to_bin in matrix_to_bin:
        temp_bin = np.zeros(15)
        temp_bin[linear_bin(value_to_bin)] = 1
        binned_matrix.append(temp_bin)
        
    return np.array(binned_matrix) 

def get_data(data_folder):
    training_csv_lines = []
    images = []
    steering_commands = []

    with open(data_folder + "training_data.csv") as csv_file:
        reader = csv.reader(csv_file, delimiter=",", quotechar="|")
        for line in reader:
            training_csv_lines.append(line)
            
    for csv_line in training_csv_lines:
        steering_command = float(csv_line[1])

        source_path = data_folder + csv_line[0]
        image = cv2.imread(source_path)
        images.append(image)

        steering_commands.append(steering_command)

        # Data augmentation
        flipped_image = cv2.flip(image, 1)
        images.append(flipped_image)
        steering_commands.append(-steering_command)

    # Grouping data into bins: https://msdn.microsoft.com/en-us/library/azure/dn913065.aspx        
    steering_commands = bin_matrix(steering_commands)
 
    return images, steering_commands


# Model is based on NVIDIA's paper: https://arxiv.org/abs/1604.07316
def get_model():
    input_image = Input(shape=(image_height, image_width, 3))

    x = input_image

    x = Lambda(lambda x: x / 255.0, input_shape=(image_height, image_width, 3))(x)
    x = Cropping2D(cropping=((90,0), (0,0)))(x)
	
    x = Conv2D(32, (5, 5), strides=(2, 2))(x)
    x = Conv2D(64, (5, 5), strides=(2, 2))(x)
    x = Conv2D(64, (3, 3), strides=(1, 1))(x)
    x = Conv2D(64, (3, 3), strides=(1, 1))(x)
    x = Flatten()(x)
    
    x = Dense(100)(x)
    x = Dropout(0.1)(x)
    x = Dense(50)(x)
    x = Dropout(0.1)(x)

    steering_output = Dense(15, activation="softmax", name="steering")(x)

    model = Model(inputs=[input_image], outputs=[steering_output])
    
    rmsprop = RMSprop(lr=0.000001)

    model.compile(optimizer=rmsprop, loss={'steering': 'categorical_crossentropy'})

    return model

training_images, training_steering_commands = get_data("../training_data/")
validation_images, validation_steering_commands = get_data("../validation_data/")

# Training data
X_train = np.array(training_images)
y_train = np.array(training_steering_commands)

# Validation data
X_valid = np.array(validation_images)
y_valid = np.array(validation_steering_commands)

model = get_model()

# Creating check point for saving only the best model
checkpoint = ModelCheckpoint("./model.h5", monitor='val_loss', verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]

history = model.fit(X_train, 
			y_train,
			validation_data=(X_valid, y_valid),
			shuffle=True,
			epochs=20,
			batch_size=128,
			verbose=1,
			callbacks=callbacks_list)

# Training and validation loss chart
plt.title("Loss")

plt.plot(history.history["loss"], color="green", label="Steering training loss")
plt.plot(history.history["val_loss"], color="blue", label="Steering validation loss")

plt.show()
