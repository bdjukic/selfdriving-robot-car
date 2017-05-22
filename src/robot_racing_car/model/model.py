#!/usr/bin/env python

import csv
import cv2
import numpy as np
import constants.py

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

training_csv_lines = []
images = []
steering_commands = []

image_width = 320
image_height = 240

with open("../training_data/training_data.csv") as csv_file:
    reader = csv.reader(csv_file, delimiter=" ", quotechar="|")
    for line in reader:
        training_csv_lines.append(line)

for csv_line in training_csv_lines:
    steering_command = int(csv_line[1])

    source_path = "../training_data/" + csv_line[0]
    image = cv2.imread(source_path)
    images.append(image)

    steering_commands.append(steering_command)

    # Data augmentation
    if steering_command == constants.TURN_LEFT_COMMAND:
        flipped_image = cv2.flip(image, 1)
        images.append(flipped_image)
        steering_commands.append(constants.TURN_RIGHT_COMMAND)
    elif steering_command == constants.TURN_RIGHT_COMMAND:
        flipped_image = cv2.flip(image, 1)
        images.append(flipped_image)
        steering_commands.append(constants.TURN_LEFT_COMMAND)
    elif steering_command == constants.MOVE_FORWARD_COMMAND:
        flipped_image = cv2.flip(image, 1)
        images.append(flipped_image)
        steering_commands.append(constants.MOVE_FORWARD_COMMAND)

X_train = np.array(images)
y_train = to_categorical(np.array(steering_commands), num_classes=4)


# Model is based on NVIDIA's paper: https://arxiv.org/abs/1604.07316
def get_model():
    model = Sequential()

    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(image_height, image_width, 3)))
    model.add(Cropping2D(cropping=((70, 0), (0, 0))))

    model.add(Conv2D(24, (5, 5), strides=(2, 2)))
    model.add(Activation("relu"))

    model.add(Conv2D(32, (5, 5), strides=(2, 2)))
    model.add(Activation("relu"))

    model.add(Conv2D(64, (5, 5), strides=(2, 2)))
    model.add(Activation("relu"))

    model.add(Conv2D(64, (3, 3), strides=(1, 1)))
    model.add(Activation("relu"))

    model.add(Conv2D(64, (3, 3), strides=(1, 1)))
    model.add(Activation("relu"))

    model.add(Flatten())

    model.add(Dense(100))
    model.add(Activation("relu"))
    model.add(Dropout(0.1))

    model.add(Dense(50))
    model.add(Activation("relu"))
    model.add(Dropout(0.1))

    model.add(Dense(10))
    model.add(Activation("relu"))
    model.add(Dropout(0.1))

    model.add(Dense(4, activation="softmax"))

    rmsprop = RMSprop(lr=0.001)
    model.compile(loss="categorical_crossentropy", optimizer=rmsprop, metrics=["accuracy"])

    return model


model = get_model()

# model.load_weights('./weights.h5')

model.fit(X_train, y_train, validation_split=0.2, shuffle=True, epochs=10, batch_size=128, verbose=1)

model.save_weights("./weights.h5")
model.save("./model.h5")
