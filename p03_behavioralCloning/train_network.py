from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers import ELU, Dense, Flatten, Lambda, Dropout
from keras.models import Sequential, Model
from keras.regularizers import l2
from keras.callbacks import History, EarlyStopping

import matplotlib.pyplot as plt

from drive import load_model


import pandas as pd
def analysis_data(csvFilePath):

    # sample = centerImage, leftImage, rightImage,steeringAngle, Throttle, break, Speed
    df = pd.read_csv(csvFilePath, header=None, usecols=[3, 4, 5, 6])
    print ("Steering Angle Max : {}, Min:{}, Avg:{}".format(df[3].max(), df[3].min(), df[3].mean()))
    print ("Throttle Max : {}, Min:{}, Avg:{}".format(df[4].max(), df[4].min(), df[4].mean()))
    print ("Break Max : {}, Min:{}, Avg:{}".format(df[5].max(), df[5].min(), df[5].mean()))
    print ("Speed Max : {}, Min:{}, Avg:{}".format(df[6].max(), df[6].min(), df[6].mean()))
    zero_steering_angles = ( (df[3][:] > -0.03) & (df[3][:] < 0.03)).sum()
    print (" # of samples with steering Angle > -0.03 and < 0.03 = {}".format(zero_steering_angles))

    return zero_steering_angles


def create_model(wt_reg=0.01, do_ratio=0.2) -> Model:
    '''
    Creates Network model similar or same as NVidia's end2end learning model.
    http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pd

    Tuned the model mostly using carnd-forums feedbacks.
    https://carnd-forums.udacity.com/questions/38548107/p3-exhausted-a-lot-of-methods-and-my-model-is-still-not-performing-well

    :return: Model
    '''

    model = Sequential()

    # Pre-processing
    # Normalize pixel through Lambda layer. pixel range from -0.5 to +0.5
    model.add(Lambda(lambda x: x /255.0 - 0.5, input_shape=(160,320,3) ))
    # Crop the image as mentioned the course video by David Silver. top 75 and bottom 25 rows of pixel
    model.add(Cropping2D(cropping=((70, 25), (0, 0)) ))

    # Convolution layers
    model.add(Convolution2D(24, 5, 5, subsample=(2,2), W_regularizer=l2(wt_reg)))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Convolution2D(36, 5, 5, subsample=(2,2), W_regularizer=l2(wt_reg)))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Convolution2D(48, 5, 5, subsample=(2,2), W_regularizer=l2(wt_reg)))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Convolution2D(64, 3, 3, W_regularizer=l2(wt_reg)))
    model.add(ELU())
    model.add(Dropout(do_ratio))
    model.add(Convolution2D(64, 3, 3, W_regularizer=l2(wt_reg)))
    model.add(ELU())
    model.add(Dropout(do_ratio))

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

def generator(samples, training_data_folder, batch_size=32, training_set=False):
    num_samples = len(samples)

    if (training_set):
        batch_size = int (batch_size /4)

    while 1:  # Loop forever so the generator never terminates
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset + batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                # sample = centerImage, leftImage, rightImage,steeringAngle, Throttle, break, Speed
                name = training_data_folder + 'IMG/' + batch_sample[0].split('/')[-1]
                center_image = cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2HSV)
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)

                if (training_set):
                    # Flip image
                    images.append(np.fliplr(center_image))
                    angles.append(-center_angle)

                    # left camera
                    left_name = training_data_folder + 'IMG/' + batch_sample[1].split('/')[-1]
                    left_image = cv2.cvtColor(cv2.imread(left_name), cv2.COLOR_BGR2HSV)
                    images.append( left_image)
                    angles.append(center_angle + 0.25)

                    # right camera
                    right_name = training_data_folder + 'IMG/' + batch_sample[2].split('/')[-1]
                    right_image = cv2.cvtColor(cv2.imread(right_name), cv2.COLOR_BGR2HSV)
                    images.append(right_image)
                    angles.append(center_angle - 0.25)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)


def train_model(model: Model, train_samples, validation_samples, training_data_folder, zero_steering_angles, num_epochs = 2, batch_size = 32) -> History:
    model.compile(loss='mse', optimizer='adam', metrics=['mse'])

    train_generator = generator(train_samples, training_data_folder, batch_size=batch_size, training_set=True)
    validation_generator = generator(validation_samples, training_data_folder, batch_size=batch_size, training_set=True)  # decided to use augmented data in validation set well

    # history = model.fit_generator(X_train, y_train, nb_epoch=num_epochs, batch_size=batch_size, validation_split=0.2,
    #           shuffle=True, verbose=1)

    earlyStoppingCallback = EarlyStopping(monitor='val_loss', min_delta=0, patience=0, verbose=0, mode='auto')

    history = model.fit_generator(train_generator, samples_per_epoch=(len(train_samples) * 4 - 3 * zero_steering_angles),   # per samples center, center-flip, left and right cameras
                                  validation_data=validation_generator,
                                  nb_val_samples=len(validation_samples), nb_epoch=num_epochs,
                                  callbacks=[earlyStoppingCallback], verbose=1)

    return history


def save_model(model: Model, file_name: str = None):
    if (file_name is None):
        file_name = "model.h5"

    print("Saving model to file {}".format(file_name))
    model.save(file_name)


import csv
from sklearn.model_selection import train_test_split


training_data_folder = '/home/girish/dev/sdcnd/CarND-Behavioral-Cloning-P3/train3/'
csv_file_name = 'driving_log.csv'
model_file_name = "model.h5"

csv_file_path = training_data_folder + csv_file_name

zero_steering_angles = analysis_data(csv_file_path)

samples = []
with open(csv_file_path) as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)


train_samples, validation_samples = train_test_split(samples, test_size=0.2)
# model = create_model()
model = load_model(model_file_name)

history_object = train_model(model, train_samples, validation_samples, training_data_folder, zero_steering_angles, 1, 32)

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


# Train 1 reverse loop, normal
# Steering Angle Max : 0.43969600000000003, Min:-0.42070280000000004, Avg:-0.014180671098950269
# Throttle Max : 0.681713, Min:0.0, Avg:0.18117340045947342
# Break Max : 0, Min:0, Avg:0.0
# Speed Max : 30.104879999999998, Min:2.3578039999999998e-07, Avg:20.325602353309833

# Train 2, reverse loop, recover
#  Steering Angle Max : 0.8765431999999999, Min:-0.9050333, Avg:0.023474771378227847
# Throttle Max : 1.0, Min:0.0, Avg:0.3139920331037975
# Break Max : 0.03549383, Min:0.0, Avg:7.540242278481013e-05
# Speed Max : 30.19205, Min:0.0009275981, Avg:12.435649089853214


#  Train 3  Normal loop, normal
# Steering Angle Max : 0.5916429, Min:-0.7815765, Avg:-0.029210627634869854
# Throttle Max : 1.0, Min:0.0, Avg:0.32958268653046385
# Break Max : 0.06442901, Min:0.0, Avg:2.9060010178857062e-05
# Speed Max : 30.19132, Min:2.183577e-07, Avg:22.008188350634697

# Train 4 Normal loop. recover
# Steering Angle Max : 0.5340347, Min:-0.9663735999999999, Avg:-0.3281584587164804
# Throttle Max : 1.0, Min:0.0, Avg:0.2420826842562849
# Break Max : 0.303454, Min:0.0, Avg:0.0016237537918994411
# Speed Max : 25.3951, Min:2.866382e-05, Avg:7.819922730765796

# train
# Steering Angle Max : 0.8765431999999999, Min:-0.9663735999999999, Avg:-0.04391709260086983
# Throttle Max : 1.0, Min:0.0, Avg:0.2663022947856477
# Break Max : 0.303454, Min:0.0, Avg:0.00016613736595215905
# Speed Max : 30.19205, Min:2.183577e-07, Avg:18.963710647114944