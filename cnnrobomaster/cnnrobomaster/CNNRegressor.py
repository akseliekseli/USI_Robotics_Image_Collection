import numpy as np

import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.layers import Conv2D, MaxPooling2D


from keras.layers.normalization import BatchNormalization

from keras import backend as K

import os

def r2_keras(y_true, y_pred):
    SS_res =  K.sum(K.square( y_true - y_pred )) 
    SS_tot = K.sum(K.square( y_true - K.mean(y_true) ) ) 
    return ( 1 - SS_res/(SS_tot + K.epsilon()) )

def main():

    batch_size = 150
    epochs =100
    # input image dimensions
    img_rows, img_cols = 28, 13
    #inputshape = X.shape[1]

    model = Sequential()
    #model.add(Dense(256, activation='relu', input_dim=366))
    model.add(Conv2D(64, (3, 3), activation='relu', input_shape = input_shape))
    #model.add(Conv2D(128, (3, 3), activation='relu'))
    #model.add(Conv2D(64, (3, 3), init='uniform'))

    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Flatten())

    model.add(Dense(512, activation='relu'))
    model.add(Dropout(0.1))

    model.add(Dense(256, activation='relu'))
    model.add(Dense(128, activation='relu'))

    model.add(Dense(1, activation='linear'))


    model.compile(loss='mean_squared_error', # one may use 'mean_absolute_error' as  mean_squared_error
                    optimizer='adam',
                    metrics=[r2_keras] # you can add several if needed
                    )

    model.summary()




    model.fit(X_train, y_train,
            batch_size=batch_size,
            epochs=epochs,
            verbose=2,
            validation_data=(X_test, y_test))
    score = model.evaluate(X_test, y_test, verbose=0)









if __name__ == '__main__':
    main()