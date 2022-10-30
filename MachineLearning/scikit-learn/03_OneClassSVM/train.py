import argparse
import os
import pickle

import joblib
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import OneClassSVM

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--nu", type=float, required=True)
    return parser.parse_args()

def train(args):
    datafile = args.data
    width    = args.width
    height   = args.height
    nu       = args.nu

    data = joblib.load(datafile)

    print('number of samples: ', len(data['data']))
    print('keys: ', list(data.keys()))
    print('image shape: ', data['data'][0].shape)

    X_train = np.array(data['data']) / 255

    # convert from [batch, width, height, channel] to [batch, width * height * channel]
    X_train = X_train.reshape(len(X_train), 3 * width * height)
    
    classifier = OneClassSVM(nu=nu,
                             kernel="rbf",
                             gamma='auto',
                             verbose=True)

    print("Start Training")
    classifier.fit(X_train)
    print("\nFinished Training")

    directory = os.path.dirname(datafile)
    filename = os.path.splitext(os.path.basename(datafile))[0]

    output = os.path.join(directory, f'{filename}_{float(nu)}_saved_model.pkl')
    pickle.dump(classifier, open(output, 'wb'))

if __name__ == '__main__':
    # parse args
    args = get_args()
    data    = args.data
    width   = args.width
    height  = args.height
    nu      = args.nu

    print("Arguments")
    print("\t   data: {}".format(data))
    print("\t  width: {}".format(width))
    print("\t height: {}".format(height))
    print("\t     nu: {}".format(float(nu)))

    train(args)
