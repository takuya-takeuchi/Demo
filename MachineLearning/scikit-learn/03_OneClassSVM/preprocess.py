import os
import argparse

from skimage import color
from skimage.io import imread
from skimage.transform import resize
import joblib

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=str, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    return parser.parse_args()

def preprocess(args):
    width   = args.width
    height  = args.height
    dataset = args.dataset

    data = dict()
    data['filename'] = []
    data['data'] = []

    name = f"{width}x{height}px.pkl"
    pklname = os.path.join(dataset, name)

    print("Start Preprocess")

    for file in os.listdir(dataset):
        if file[-3:] in {'jpg', 'png'}:
            print(f"\t{file}")
            path = os.path.join(dataset, file)
            im = imread(path)
            if len(im.shape) == 2:
                im = color.gray2rgb(im)
            im = resize(im, (width, height)) #[:,:,::-1]
            data['filename'].append(path)
            data['data'].append(im)

    joblib.dump(data, pklname)

    print("Finished Preprocess")

if __name__ == '__main__':
    args = get_args()
    width   = args.width
    height  = args.height
    dataset = args.dataset

    print("Arguments")
    print("\t  width: {}".format(width))
    print("\t height: {}".format(height))
    print("\tdataset: {}".format(dataset))

    preprocess(args)
