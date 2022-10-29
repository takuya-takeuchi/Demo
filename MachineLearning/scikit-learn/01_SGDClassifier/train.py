import argparse
import os
import pickle

from collections import Counter
import joblib
import numpy as np

from sklearn.linear_model import SGDClassifier
from sklearn.model_selection import train_test_split

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    return parser.parse_args()

def train(args):
    datafile = args.data
    width    = args.width
    height   = args.height

    data = joblib.load(datafile)

    print('number of samples: ', len(data['data']))
    print('keys: ', list(data.keys()))
    print('image shape: ', data['data'][0].shape)
    print('labels:', np.unique(data['label']))
    
    Counter(data['label'])

    X = np.array(data['data']) / 255
    y = np.array(data['label'])

    X_train, X_test, y_train, y_test = train_test_split(
        X, 
        y, 
        test_size=0.2, 
        shuffle=True,
        random_state=42
    )

    # convert from [batch, width, height, channel] to [batch, width * height * channel]
    X_train = X_train.reshape(len(X_train), 3 * width * height)
    X_test = X_test.reshape(len(X_test), 3 * width * height)
    
    classifier = SGDClassifier(random_state=42,
                               max_iter=1000,
                               tol=1e-3)

    print("Start Training")
    classifier.fit(X_train, y_train)
    print("Finished Training")

    print('Percentage correct: ', 100 * np.sum(classifier.predict(X_test) == y_test) / len(y_test))

    directory = os.path.dirname(datafile)
    filename = os.path.splitext(os.path.basename(datafile))[0]

    output = os.path.join(directory, f'{filename}_saved_model.pkl')
    pickle.dump(classifier, open(output, 'wb'))

if __name__ == '__main__':
    # parse args
    args = get_args()
    data    = args.data
    width   = args.width
    height  = args.height

    print("Arguments")
    print("\t   data: {}".format(data))
    print("\t  width: {}".format(width))
    print("\t height: {}".format(height))

    train(args)
