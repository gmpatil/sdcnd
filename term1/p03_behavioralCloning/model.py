from keras.layers.convolutional import Convolution2D, Cropping2D, MaxPooling2D
from keras.layers import ELU, Dense, Flatten, Lambda, Dropout
from keras.models import Sequential, Model, load_model
from keras.regularizers import l2
from keras.callbacks import History, EarlyStopping

import matplotlib.pyplot as plt

import pandas as pd
import csv

from sklearn.model_selection import train_test_split

def analyse_data(csvFilePath):

    # sample = centerImage, leftImage, rightImage,steeringAngle, Throttle, break, Speed
    df = pd.read_csv(csvFilePath, header=None, usecols=[3, 4, 5, 6])
    print("Number of samples {}".format(df[3].count()))
    print ("Steering Angle Max : {}, Min:{}, Avg:{}".format(df[3].max(), df[3].min(), df[3].mean()))
    print ("Throttle Max : {}, Min:{}, Avg:{}".format(df[4].max(), df[4].min(), df[4].mean()))
    print ("Break Max : {}, Min:{}, Avg:{}".format(df[5].max(), df[5].min(), df[5].mean()))
    print ("Speed Max : {}, Min:{}, Avg:{}".format(df[6].max(), df[6].min(), df[6].mean()))
    zero_steering_angles = ( (df[3][:] > -0.03) & (df[3][:] < 0.03)).sum()
    print (" # of samples with steering Angle > -0.03 and < 0.03 = {}".format(zero_steering_angles))

    return zero_steering_angles


def hist_steering_angles(samplesArr):
    '''
    Plots histogram of Steering Angles

    :param samplesArr: array of samples with steering angles
    :return: None
    '''
    samplesArr[:, 3] = samplesArr[:, 3].astype(float)
    sa = samplesArr[:, 3].astype(float)

    import matplotlib.pyplot as plt
    import random

    plt.hist(sa, bins=100)

    plt.show()


def reduce_brightness(image):
    '''
    Adjusts the brightness of the input image. Darkens randomly between 20% to 80%

    https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.8mcdtb70w

    :param image: image who's brightness to be reduced
    :return: image with brightness reduced.
    '''

    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)
    # random_bright = .5+np.random.uniform()
    image1[:,:,2] = image1[:,:,2]* np.random.uniform(.20, .80)  #  < 1 darken. (> 1 and  < 2) increase brightness
    # image1[:, :, 2] = image1[:, :, 2] * 0.5
    image1[:,:,2][image1[:,:,2]>255]  = 255
    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1


def add_shade(image):
    '''
    Add shade to the left or right part of the image

    :param image: input image on which dark shade to be added
    :return: updated image
    '''
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)

    # Add random shadow as a vertical slice of image
    ym, xm = image.shape[0], image.shape[1]
    [x0, x1] = np.random.choice(xm, 2, replace=False)

    m = ( (ym * 1.0) / (x1 - x0))
    c = (-1.0 * m * x0)
    shadeSide = np.random.choice([0, 1])

    for y in range (ym):
        if (shadeSide == 0):
            image1[y, : int( (y - c) / m), 1] = image1[y, : int((y - c) / m), 1] * 0.5
            image1[y, : int( (y - c) / m), 2] = image1[y, : int((y - c) / m), 2] * 0.25
        else:
            image1[y, int( (y - c) / m):, 1] = image1[y, int((y - c) / m):, 1] * 0.5
            image1[y, int( (y - c) / m):, 2] = image1[y, int((y - c) / m):, 2] * 0.25

    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1


def filter_some_center_steering_angles(samplesArr):
    '''
    If samples contain more than 30% with straight steering angles, excess is removed.
    Angles  > -0.03 and < +0.03 are considered straight steering angles.

    :param samplesArr: input samples
    :return: samples with excess straight steering angles removed.

    '''
    centerSamples = samplesArr[(samplesArr[:, 3].astype(float) > -0.03) & (samplesArr[:, 3].astype(float) < +0.03)]
    leftSamples = samplesArr[(samplesArr[:, 3].astype(float) >= +0.03)]
    rightSamples = samplesArr[(samplesArr[:, 3].astype(float) <= -0.03)]

    print("Center {}  Left {}  Right {}".format(len(centerSamples), len(leftSamples), len(rightSamples)))

    if (centerSamples.shape[0] > int(((len(leftSamples) + len(rightSamples)) / 2))):
        centerSamples = centerSamples[np.random.choice(centerSamples.shape[0],
                                                       int(((len(leftSamples) + len(rightSamples)) / 2)), replace=False), :]

    print("Center after filtering {}  {}".format(len(centerSamples), centerSamples.shape))

    return np.concatenate((centerSamples, leftSamples, rightSamples))


def augment_data(samplesArr):
    '''
    Augment the sample data by using left and right camera images and then creating 3 additional flipped image samples.
    Thus adding 5 additional samples for one increasing sample data size by 500%.

    :param samplesArr: input sample, mainly with center camera image and steering angles.
    :return: samples data with augmented data
    '''
    ret = []

    for i in range(samplesArr.shape[0]):
        ret.append([samplesArr[i,0], float(samplesArr[i,3]), False])  # center image
        ret.append([samplesArr[i,1], (float(samplesArr[i,3]) + 0.25), False]) # left image
        ret.append([samplesArr[i,2], (float(samplesArr[i,3]) - 0.25), False]) # right image

        # Flip the images
        ret.append([samplesArr[i,0], float(samplesArr[i,3]) * -1.0 , True])  # center image
        ret.append([samplesArr[i,1], (float(samplesArr[i,3]) + 0.25) * -1.0 , True]) # left image
        ret.append([samplesArr[i,2], (float(samplesArr[i,3]) - 0.25) * -1.0 , True]) # right image

    return ret


def create_model(wt_reg=0.01, do_ratio=0.2) -> Model:
    '''
    Creates Network model similar or same as NVidia's end2end learning model.
    http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pd

    Tuned the model mostly using carnd-forums feedbacks.
    https://carnd-forums.udacity.com/questions/38548107/p3-exhausted-a-lot-of-methods-and-my-model-is-still-not-performing-well

    Removed 2 convolution layers from original NVIDIA model and added MaxPool layers. Used ELU activations instead Tanh or ReLU.

    :return: Model
    '''

    model = Sequential()

    # Pre-processing
    # Normalize pixel through Lambda layer. pixel range from -0.5 to +0.5
    model.add(Lambda(lambda x: x /255.0 - 0.5, input_shape=(160,320,3) ))
    # Crop the image as mentioned the course video by David Silver. top 75 and bottom 25 rows of pixel
    model.add(Cropping2D(cropping=((75, 25), (0, 0)) ))

    # Convolution layers
    model.add(Convolution2D(16, 3, 3, subsample=(1,1), W_regularizer=l2(wt_reg)))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Convolution2D(32, 3, 3, subsample=(1,1), W_regularizer=l2(wt_reg)))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Convolution2D(64, 3, 3, subsample=(2,2), W_regularizer=l2(wt_reg)))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(ELU())
    model.add(Dropout(do_ratio))

    # Remove below 2 convolution layers
    # model.add(Convolution2D(64, 3, 3, W_regularizer=l2(wt_reg)))
    # model.add(MaxPooling2D(pool_size=(2, 2)))
    # model.add(ELU())
    # model.add(Dropout(do_ratio))
    # Cropping top 75 rows instead 70 causing this layer to become -ve.
    # model.add(Convolution2D(64, 3, 3, W_regularizer=l2(wt_reg)))
    # model.add(ELU())
    # model.add(Dropout(do_ratio))

    model.add(Flatten())

    model.add(Dense(100))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Dense(50))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Dense(10))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Dense(1, activation='linear'))

    return model


import cv2
import numpy as np
import sklearn

def generator(samples, training_data_folder, batch_size=32):
    '''
    Keras generator function, allow us to feed training and validation data without loading all the images in to the memory.

    :param samples: input samples image file names and steering angles
    :param training_data_folder: folder from where images to be loaded
    :param batch_size: batch size
    :return:
    '''
    num_samples = len(samples)

    while 1:  # Loop forever so the generator never terminates
        sklearn.utils.shuffle(samples)


        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset: offset + batch_size]

            images = []
            angles = []

            for batch_sample in batch_samples:
                name = training_data_folder + 'IMG/' + batch_sample[0].split('/')[-1]
                image = cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB)
                steeringAngle = float(batch_sample[1])  # 3

                if (batch_sample[2]): # if flip image
                    image = np.fliplr(image)

                choice = np.random.choice(4, 1)
                if (choice == 1):
                    images.append(reduce_brightness(image))   # 25% images will have brightness altered
                    angles.append(steeringAngle)
                elif (choice == 2):
                    images.append(add_shade(image))  # 25% images will have shade added
                    angles.append(steeringAngle)
                else:
                    images.append(image)
                    angles.append(steeringAngle)

            X_train = np.array(images)
            y_train = np.array(angles)
            yield X_train, y_train


def train_model(model: Model, train_samples, validation_samples, training_data_folder, zero_steering_angles, num_epochs = 2, batch_size = 32) -> History:
    model.compile(loss='mse', optimizer='adam', metrics=['mse'])

    train_generator = generator(train_samples, training_data_folder, batch_size=batch_size)
    validation_generator = generator(validation_samples, training_data_folder, batch_size=batch_size)  # Using augmented data in validation set well

    earlyStoppingCallback = EarlyStopping(monitor='val_loss', min_delta=0, patience=0, verbose=0, mode='auto')

    history = model.fit_generator(train_generator, samples_per_epoch=(len(train_samples)),
                                  validation_data=validation_generator,
                                  nb_val_samples=len(validation_samples), nb_epoch=num_epochs,
                                  callbacks=[earlyStoppingCallback], verbose=1)

    return history


def save_model(model: Model, file_name: str = None):
    if (file_name is None):
        file_name = "model.h5"

    print("Saving model to file {}".format(file_name))
    model.save(file_name)


def evaluate_model(model_file_name, training_data_folder, csv_file_path ):
    '''
    Evaluate model performance against valuation or test data set.

    :param model_file_name:
    :validationSet
    :return: model.metrics like 'loss', 'mean_squared_error'
    '''

    samples = []

    with open(csv_file_path) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append([line[0], line[3], False])

    samplesArr = np.asarray(samples)
    model = load_model(model_file_name)
    validation_generator = generator(samplesArr, training_data_folder, batch_size=32)
    test_loss = model.evaluate_generator(generator=validation_generator, val_samples=len(samples))

    print(model.metrics_names, test_loss)

    return test_loss


if __name__ == '__main__':

    training_data_folder = '/home/girish/dev/sdcnd/CarND-Behavioral-Cloning-P3/train3/'
    csv_file_name = 'driving_log.csv'
    model_file_name = "model.h5"

    csv_file_path = training_data_folder + csv_file_name

    # test_loss = evaluate_model(model_file_name, training_data_folder, csv_file_path)

    zero_steering_angles = analyse_data(csv_file_path)

    samples = []
    with open(csv_file_path) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)

    samplesArr = np.asarray(samples)

    # hist_steering_angles(samplesArr)

    samplesArr = filter_some_center_steering_angles(samplesArr)

    sampleList = augment_data(samplesArr)
    print ("Samples size after data augmentation: {}".format(len(sampleList)))

    train_samples, validation_samples = train_test_split(sampleList, test_size=0.2)
    print ("Training sample size {} \nValidation sample size {}".format(len(train_samples), len(validation_samples)))

    newModel = False
    if (newModel):
        model = create_model()
    else:
        model = load_model(model_file_name)

    history_object = train_model(model, train_samples, validation_samples, training_data_folder, zero_steering_angles, 2, 32)

    save_model(model, model_file_name)

    ### print the keys contained in the history object
    print(history_object.history.keys())

    ### plot the training and validation loss for each epoch
    plt.plot(history_object.history['loss'])
    plt.plot(history_object.history['val_loss'])
    plt.title('model mean squared error loss')
    plt.ylabel('mean squared error loss')
    plt.xlabel('epoch')
    plt.legend(['training set', 'validation set'], loc='upper right')
    plt.show()

