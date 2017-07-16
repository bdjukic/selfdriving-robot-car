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

def linear_bin(a):
    a = a + 1
    b = round(a / (2.0/14.0))
    return int(b)

def bin_Y(Y):
    d = []
    for y in Y:
        arr = np.zeros(15)
        arr[linear_bin(y)] = 1
        d.append(arr)
    return np.array(d) 

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
        #flipped_image = cv2.flip(image, 1)
        #images.append(flipped_image)
        #steering_commands.append(-steering_command)
        
    steering_commands = bin_Y(steering_commands)
 
    return images, steering_commands


# Model is based on NVIDIA's paper: https://arxiv.org/abs/1604.07316
def get_model():
    input_image = Input(shape=(image_height, image_width, 3))

    x = input_image

    x = Lambda(lambda x: x / 255.0, input_shape=(image_height, image_width, 3))(x)
	
    x = Conv2D(32, (5, 5), strides=(2, 2))(x)
    x = Conv2D(64, (5, 5), strides=(2, 2))(x)
    x = Conv2D(64, (3, 3), strides=(1, 1))(x)
    x = Conv2D(64, (3, 3), strides=(1, 1))(x)
    x = Flatten()(x)
    
    x = Dense(100)(x)
    x = Dropout(0.1)(x)
    x = Dense(50)(x)
    x = Dropout(0.1)(x)

    steering_output = Dense(15, activation="softmax", name="steering_output")(x)

    model = Model(inputs=[input_image], outputs=[steering_output])
    
    rmsprop = RMSprop(lr=0.000001)

    model.compile(optimizer=rmsprop,
			      loss={'steering_output': 'categorical_crossentropy'})

    return model

training_images, training_steering_commands = get_data("../training_data/")
validation_images, validation_commands = get_data("../validation_data/")

# Training data
X_train = np.array(training_images)
y_train = np.array(training_steering_commands)

# Validation data
X_valid = np.array(validation_images)
y_valid = np.array(validation_commands)

model = get_model()

# Creating check point for saving only the best model's weights
checkpoint = ModelCheckpoint("./weights.h5", monitor='val_loss', verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]

history = model.fit(X_train, 
			y_train,
			validation_data=(X_valid, y_valid),
			shuffle=True,
			epochs=20,
			batch_size=128,
			verbose=1,
			callbacks=callbacks_list)

model.load_weights('./weights.h5')
model.save("./model.h5")

# Make predictions
train_predictions = model.predict(X_train)
test_predictions = model.predict(X_valid)

plt.title("Loss")
plt.plot(history.history["loss"], color="green", label="Train")
plt.plot(history.history["val_loss"], color="red", label="Validation")

plt.show()
