'''
Based on:
https://gist.github.com/harvitronix/9ea862171b5a8188b7e6b5e587a3f6ad#file-train-py
'''

import csv, random, numpy as np
import matplotlib.pyplot as plt
from keras.models import load_model, Sequential
from keras.layers import Dense, Dropout, Flatten, Lambda, Activation
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.preprocessing.image import img_to_array, load_img, random_shift
import cv2
from matplotlib import pyplot
from keras.callbacks import ModelCheckpoint

def resize(img):
    """
    Resizes the images in the supplied tensor to the original dimensions of the NVIDIA model (66x200)
    """
    from keras.backend import tf as ktf
    return ktf.image.resize_images(img, [100, 100])


def model_nvidia(shape):
    
    model = Sequential()
    
    # Cropping image
    #model.add(Lambda(lambda imgs: imgs[:,:,:,:], input_shape=shape))

    # Normalise the image - center the mean at 0
    model.add(Lambda(lambda imgs: (imgs/255.0) - .5, input_shape=shape))
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


def model(load, shape, checkpoint=None):

    # Return a model from file or to train on

    if load: return load_model(checkpoint)

    conv_layers, dense_layers = [32, 32, 64, 128], [1024, 512]
    
    model = Sequential()

    # Cropping image
    #model.add(Lambda(lambda imgs: imgs[:,:,:,:], input_shape=shape))

    # Normalise the image - center the mean at 0
    model.add(Lambda(lambda imgs: (imgs/255.0) - .5, input_shape=shape))
    #model.add(Lambda(resize))

    model.add(Conv2D(32, (3, 3), activation='elu'))
    model.add(MaxPooling2D())

    for cl in conv_layers:
        model.add(Conv2D(cl, (3, 3), activation='elu'))
        model.add(MaxPooling2D())

    model.add(Flatten())

    for dl in dense_layers:
        model.add(Dense(dl, activation='elu'))
        model.add(Dropout(0.5))

    model.add(Dense(1, activation='linear'))

    model.summary()

    model.compile(loss='mse', optimizer="adam")

    return model
    
def get_X_y(data_file):
    """Read the log file and turn it into X/y pairs. Add an offset to left images, remove from right images."""
    X, y = [], []
    steering_offset = 0.4
    with open(data_file) as fin:
        for center_img, left_img, right_img, steering_angle, _, _, speed in csv.reader(fin):
            if float(speed) < 20: continue  # throw away low-speed samples
            X += [center_img.strip(), left_img.strip(), right_img.strip()]
            y += [float(steering_angle), float(steering_angle) + steering_offset, float(steering_angle) - steering_offset]
    return X, y

def flip_image(img):
    
    # Returns a horizontally flipped image
    
    return cv2.flip(img, 1)

def process_image(path, steering_angle, augment, shape=(100,100)):
    """Process and augment an image."""
    image = load_img(path, target_size=shape)

    if augment and random.random() < 0.5:
        image = random_darken(image)  # before numpy'd

    image = img_to_array(image)
        
    if augment:
        image = random_shift(image, 0, 0.2, 0, 1, 2)  # only vertical
        if random.random() < 0.5:
            image = flip_image(image)
            steering_angle = -steering_angle

    #image = (image / 255. - .5).astype(np.float32) # Normalize image to +-0.5
    
    return image, steering_angle

def random_darken(image):
    """Given an image (from Image.open), randomly darken a part of it."""
    w, h = image.size

    # Make a random box.
    x1, y1 = random.randint(0, w), random.randint(0, h)
    x2, y2 = random.randint(x1, w), random.randint(y1, h)

    # Loop through every pixel of our box (*GASP*) and darken.
    for i in range(x1, x2):
        for j in range(y1, y2):
            new_value = tuple([int(x * 0.5) for x in image.getpixel((i, j))])
            image.putpixel((i, j), new_value)
    return image

def _generator(batch_size, X, y):
    """Generate batches of training data forever."""
    while 1:
        batch_X, batch_y = [], []
        for i in range(batch_size):
            sample_index = random.randint(0, len(X) - 1)
            sa = y[sample_index]
            image, sa = process_image(X[sample_index], sa, augment=True)
            batch_X.append(image)
            batch_y.append(sa)
        yield np.array(batch_X), np.array(batch_y)

def train():
    """Load our network and our data, fit the model, save it."""
    net = model(load=False, shape=(100, 100, 3))
    #net = model_nvidia(shape=(100, 100, 3))
    X, y = get_X_y('driving_log.csv')

    history = net.fit_generator(_generator(64, X, y), steps_per_epoch=len(X)/64, epochs=5)
    net.save('model.h5')

    print(history.history.keys())

    # summarize history for loss
    plt.plot(history.history['loss'])
    plt.title('model loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.legend(['train', 'test'], loc='upper left')
    plt.show()

if __name__ == '__main__':
    train()
