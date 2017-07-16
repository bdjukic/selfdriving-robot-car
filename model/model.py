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

def get_data(data_folder):
    training_csv_lines = []
    images = []
    steering_commands = []
    throttle_values = []

    with open(data_folder + "training_data.csv") as csv_file:
        reader = csv.reader(csv_file, delimiter=",", quotechar="|")
        for line in reader:
            training_csv_lines.append(line)
            
    for csv_line in training_csv_lines:
        steering_command = float(csv_line[1])
        throttle_value = float(csv_line[2])

        source_path = data_folder + csv_line[0]
        image = cv2.imread(source_path)
        images.append(image)

        steering_commands.append(steering_command)
        throttle_values.append(throttle_value)

        # Data augmentation
        flipped_image = cv2.flip(image, 1)
        images.append(flipped_image)
        steering_commands.append(-steering_command)
        throttle_values.append(throttle_value)
 
    return images, steering_commands, throttle_values


# Model is based on NVIDIA's paper: https://arxiv.org/abs/1604.07316
def get_model():
    input_image = Input(shape=(image_height, image_width, 3), name="input_image")

    x = input_image

    x = Lambda(lambda x: x / 255.0 - 0.5, input_shape=(image_height, image_width, 3))(x)
    x = Cropping2D(cropping=((40, 0), (0, 0)))(x)
	
    x = Conv2D(24, (5, 5), strides=(2, 2), activation="relu")(x)
    x = Conv2D(36, (5, 5), strides=(2, 2), activation="relu")(x)
    x = Conv2D(48, (5, 5), strides=(2, 2), activation="relu")(x)
    x = Conv2D(64, (3, 3), strides=(1, 1), activation="relu")(x)
    x = Conv2D(64, (3, 3), strides=(1, 1), activation="relu")(x)
    x = Flatten()(x)
    
    x = Dense(100, activation="relu")(x)
    x = Dropout(0.1)(x)
    x = Dense(50, activation="relu")(x)
    x = Dropout(0.1)(x)

    steering_output = Dense(1, activation="relu", name="steering_output")(x)
    throttle_output = Dense(1, activation="relu", name="throttle_output")(x)

    model = Model(inputs=[input_image], outputs=[steering_output, throttle_output])

    model.compile(optimizer="rmsprop",
			      loss={'steering_output': 'categorical_crossentropy', 'throttle_output': 'mean_absolute_error'})

    return model

training_images, training_steering_commands, training_throttle_values = get_data("../training_data/")
validation_images, validation_commands, validation_throttle_values = get_data("../validation_data/")

# Training data
X_train = np.array(training_images)
y_train = np.array(training_steering_commands)
z_train = np.array(training_throttle_values)

# Validation data
X_valid = np.array(validation_images)
y_valid = np.array(validation_commands)
z_valid = np.array(validation_throttle_values)

model = get_model()

# Creating check point for saving only the best model's weights
checkpoint = ModelCheckpoint("./weights.h5", monitor='val_loss', verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]

history = model.fit(X_train, 
			[y_train, z_train],
			validation_data=(X_valid, [y_valid, z_valid]),
			shuffle=True,
			epochs=10,
			batch_size=128,
			verbose=1,
			callbacks=callbacks_list)

model.load_weights('./weights.h5')
model.save("./model.h5")

# Make predictions
train_predictions = model.predict(X_train)
test_predictions = model.predict(X_valid)

plt.title("Loss")
plt.plot(history.history["steering_output_loss"], color="green", label="Train")
plt.plot(history.history["throttle_output_loss"], color="blue", label="Train")
plt.plot(history.history["val_steering_output_loss"], color="red", label="Validation")
plt.plot(history.history["val_throttle_output_loss"], color="black", label="Validation")

plt.show()
