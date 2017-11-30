# coding: utf-8

import numpy as np

from keras.layers import Conv2D, Dense, Activation, Flatten, MaxPooling2D
from keras.layers.normalization import BatchNormalization
from keras.models import Sequential
from keras import backend as K


def build_visual_encoder():
    model = Sequential()
    model.add(Conv2D(32, (3,3), input_shape=(224, 224, 3)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D())
    model.add(BatchNormalization())
    model.add(Conv2D(32, (3,3)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D())
    model.add(BatchNormalization())
    model.add(Flatten())
    model.add(Dense(32, activation='relu'))
    model.compile(optimizer='rmsprop',
                  loss='mse')
    return model

def build_action_maker():
    model = Sequential()
    model.add(Dense(16, input_shape=(32,)))
    model.compile(optimizer='rmsprop',
                  loss='mse')
    return model

def build_action_decoder():
    model = Sequential()
    model.add(Dense(8, input_shape=(16,)))
    model.compile(optimizer='rmsprop',
                  loss='mse')
    return model
