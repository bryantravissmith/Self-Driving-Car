import cv2
import csv
import numpy as np
from keras.models import Sequential, Model
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from keras.layers import Cropping2D, Lambda
import matplotlib.pyplot as plt


def read_data(dir, corrections, skip_first=False):
    lines = []
    images = []
    measurements = []
    with open('{}/driving_log.csv'.format(dir)) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            lines.append(line)

    if skip_first:
        lines = lines[1:]

    for line in lines:
        for i in range(3):
            filename = line[i].split('/')[-1]
            current_path = '{}/IMG/'.format(dir) + filename
            image = cv2.imread(current_path)
            image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            images.append(image)

            measurement = float(line[3]) + corrections[i]
            measurements.append(measurement)

    return images, measurements

def create_data():
    images = []
    measurements = []


    corrections = {
        0:0,
        1:+0.2,
        2:-0.2
    }

    img, meas = read_data('data', corrections)
    images = images + img
    measurements = measurements + meas

    img, meas = read_data('data_udacity', corrections, skip_first=True)
    images = images + img
    measurements = measurements + meas

    measurements = np.array(measurements)
    images = np.array(images)
    images = np.concatenate([images,np.fliplr(images)])
    measurements = np.concatenate([measurements, -measurements])

    return images, measurements

def create_nvidia_model():
    model = Sequential()
    model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
    model.add(Cropping2D(
        cropping=((50,20), (0,0)),
        data_format='channels_last'
    ))
    model.add(Conv2D(3, (5,5), strides=1, activation='relu'))
    model.add(Conv2D(24, (5,5), strides=2, activation='relu'))
    model.add(Conv2D(36, (5,5), strides=2, activation='relu'))
    model.add(Conv2D(48, (5,5), strides=2, activation='relu'))
    model.add(Conv2D(64, (3,3), strides=2, activation='relu'))
    model.add(Conv2D(64, (3,3), strides=2, activation='relu'))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))
    model.compile(loss='mse', optimizer='adam')
    return model

if __name__ == "__main__":
    images, measurements = create_data()
    model = create_nvidia_model()
    history = model.fit(
        images,
        measurements,
        validation_split=0.2,
        shuffle=True,
        epochs=10,
        batch_size=128
    )
    model.save('model.h5')

    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('model mean squared error loss')
    plt.ylabel('mean squared error loss')
    plt.xlabel('epoch')
    plt.legend(['training set', 'validation set'], loc='upper right')
    plt.savefig('validation_plot.png')
