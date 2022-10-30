import argparse
import os
import pickle

import numpy as np
from skimage import color
from skimage.io import imread
from skimage.transform import resize

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=True)
    parser.add_argument("--image_root", type=str, required=True)
    parser.add_argument("--good", nargs='+', default=[], required=False)
    parser.add_argument("--abnormal", nargs='+', default=[], required=False)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    return parser.parse_args()

def eval(args):
    image_root = args.image_root
    good       = args.good
    abnormal   = args.abnormal
    model      = args.model
    width      = args.width
    height     = args.height
    
    model = pickle.load(open(model, 'rb'))

    x_good_test = []
    y_good_test = []
    x_abnormal_test = []
    y_abnormal_test = []

    for g in good:
        root = os.path.join(image_root, g)
        for file in os.listdir(root):
            if file[-3:] in {'jpg', 'png'}:
                path = os.path.join(root, file)
                im = imread(path)
                if len(im.shape) == 2:
                    im = color.gray2rgb(im)
                im = resize(im, (width, height)) #[:,:,::-1]
                x = np.array(im) / 255
                # convert from [batch, width, height, channel] to [width * height * channel]
                x = x.reshape(3 * width * height)
                x_good_test.append(x)
                y_good_test.append(1)
    
    for a in abnormal:
        root = os.path.join(image_root, a)
        for file in os.listdir(root):
            if file[-3:] in {'jpg', 'png'}:
                path = os.path.join(root, file)
                im = imread(path)
                if len(im.shape) == 2:
                    im = color.gray2rgb(im)
                im = resize(im, (width, height)) #[:,:,::-1]
                x = np.array(im) / 255
                # convert from [batch, width, height, channel] to [width * height * channel]
                x = x.reshape(3 * width * height)
                x_abnormal_test.append(x)
                y_abnormal_test.append(-1)

    print("Start Evaluation")
    if len(x_good_test) > 0:
        y_good_predict = model.predict(x_good_test)
    if len(x_abnormal_test) > 0:
        y_abnormal_predict = model.predict(x_abnormal_test)
    print("Finished Evaluation")

    print('Percentage correct:')
    if len(x_good_test) > 0:
        # print(y_good_predict)
        print('\t    good: ', 100 * np.sum(y_good_predict == x_good_test) / len(x_good_test))
    else:
        print('\t    good: N/A')
    if len(x_abnormal_test) > 0:
        # print(y_abnormal_predict)
        print('\tabnormal: ', 100 * np.sum(y_abnormal_predict == y_abnormal_test) / len(y_abnormal_test))
    else:
        print('\tabnormal: N/A')

if __name__ == '__main__':
    # parse args
    args = get_args()
    image_root = args.image_root
    good       = args.good
    abnormal   = args.abnormal
    model      = args.model
    width      = args.width
    height     = args.height

    print("Arguments")
    print("\t  image_root: {}".format(image_root))
    print("\t        good: {}".format(good))
    print("\t    abnormal: {}".format(abnormal))
    print("\t       model: {}".format(model))
    print("\t       width: {}".format(width))
    print("\t      height: {}".format(height))

    eval(args)
