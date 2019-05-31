#!/usr/bin/env python

'''
* Deep learning training script *********************
 
 For training a deep learning model with image and steering data.
 The output of the model is a corresponding steering angle.

 Based on:
 https://gist.github.com/harvitronix/9ea862171b5a8188b7e6b5e587a3f6ad#file-train-py
 
 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

import csv, random, numpy as np
from keras.models import load_model, Sequential
from keras.layers import Dense, Dropout, Flatten, Lambda, Activation
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.preprocessing.image import img_to_array, load_img, random_shift
from keras.callbacks import ModelCheckpoint
import matplotlib.pyplot as plt
import cv2


def model_nvidia(shape):
    
    model = Sequential()
    
    # Cropping image
    #model.add(Lambda(lambda imgs: imgs[:,:,:,:], input_shape=shape))

    # Normalise the image - center the mean at 0
    model.add(Lambda(lambda imgs: (imgs/255.0) - .1, input_shape=shape))
    model.add(Lambda(resize))
    
    # We have a series of 3 5x5 convolutional layers with a stride of 2x2
    model.add(Conv2D(24, (5, 5), strides=(2, 2)))
    model.add(Activation('elu'))

    model.add(Conv2D(36, (5, 5), strides=(2, 2)))
    model.add(Activation('elu'))

    model.add(Conv2D(48, (5, 5), strides=(2, 2)))  
    model.add(Activation('elu'))    

    model.add(Conv2D(64, (3, 3), strides=(1, 1))) 
    model.add(Activation('elu'))    

    model.add(Conv2D(64, (3, 3), strides=(1, 1))) 
    model.add(Activation('elu'))    

    model.add(Dropout(0.5))
    model.add(Flatten())
    
    # Fully connected layers
    model.add(Dense(100))    
    model.add(Activation('elu'))

    model.add(Dense(50))    
    model.add(Activation('elu'))
    
    model.add(Dense(10))    
    model.add(Activation('elu'))    

    # Output layer
    model.add(Dense(1))

    model.summary()
    
    model.compile(loss = "mse", optimizer = "adam")

    return model 


def model(shape):
    
    # Create a new sequential network model
    model = Sequential()    

    # Normalise the input image data
    model.add(Lambda(lambda imgs: (imgs/255.0) - .1, input_shape=shape))

    # Convolutional layers
    model.add(Conv2D(32, (3, 3), activation='elu'))
    model.add(MaxPooling2D())

    model.add(Conv2D(32, (3, 3), activation='elu'))
    model.add(MaxPooling2D())

    model.add(Conv2D(32, (3, 3), activation='elu'))
    model.add(MaxPooling2D())

    model.add(Conv2D(64, (3, 3), activation='elu'))
    model.add(MaxPooling2D())

    model.add(Conv2D(128, (3, 3), activation='elu'))
    model.add(MaxPooling2D())

    model.add(Flatten())

    # Fully connected layers
    model.add(Dense(1024, activation='elu'))
    model.add(Dropout(0.5))

    model.add(Dense(512, activation='elu'))
    model.add(Dropout(0.5))

    # Output layer
    model.add(Dense(1, activation='linear'))

    # Print model summary
    model.summary()

    # Compile using mean square error and adam optimizer
    model.compile(loss='mse', optimizer="adam")

    return model
    

def get_data_from_file(log_file):

    img_list, steering_list = [], []

    with open(log_file) as f:
        for center_img, steering_angle, speed in csv.reader(f):
            if float(speed) <= 0.0: continue # Skip low speed data
            img_list += [center_img.strip()]
            steering_list += [float(steering_angle)]
    return img_list, steering_list


def resize(img):

    from keras.backend import tf as ktf
    return ktf.image.resize_images(img, [100, 100])


def flip_image(img):      

    return cv2.flip(img, 1)


def random_shadow(image):

    # Get image size
    w, h = image.size

    # Make a random box.
    x1, y1 = random.randint(0, w), random.randint(0, h)
    x2, y2 = random.randint(x1, w), random.randint(y1, h)

    # Darken the pixels in the box
    for i in range(x1, x2):
        for j in range(y1, y2):
            new_value = tuple([int(x * 0.5) for x in image.getpixel((i, j))])
            image.putpixel((i, j), new_value)

    return image


# Convert image to array and augment training data
def process_image(img_path, steering_angle, augment, shape=(100,100)):

    image = load_img(img_path, target_size=shape)

    if augment and random.random() < 0.5:
        image = random_shadow(image)

    image = img_to_array(image)
        
    if augment:
        image = random_shift(image, 0, 0.2, 0, 1, 2)  # only vertical
        if random.random() < 0.5:
            image = flip_image(image)
            steering_angle = -steering_angle
    
    return image, steering_angle


def batch_generator(batch_size, img_list, steering_input_list):

    while 1:

        batch_img, batch_steering = [], []

        for i in range(batch_size):
            sample_index = random.randint(0, len(img_list) - 1)
            steering_angle = steering_input_list[sample_index]
            image, steering_angle = process_image(img_list[sample_index], steering_angle, augment=True)
            batch_img.append(image)
            batch_steering.append(steering_angle)

        yield np.array(batch_img), np.array(batch_steering)


def train():
    
    # Settings
    batch_size = 32
    num_epochs = 5
    
    net = model(shape=(100, 100, 3)) # Set model
    img_list, steering_input_list = get_data_from_file('training_data.csv') # Load data

    history = net.fit_generator(batch_generator(batch_size, img_list, steering_input_list), steps_per_epoch=len(img_list)/batch_size, epochs=num_epochs)
    net.save('model.h5') # Save

    # summarize history for loss
    plt.plot(history.history['loss'])
    plt.title('model loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.legend(['Training'], loc='upper left')
    plt.show()


if __name__ == '__main__':
    train()
