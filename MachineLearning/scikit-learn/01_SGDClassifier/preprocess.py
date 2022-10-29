import os
import argparse

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
    data['label'] = []
    data['filename'] = []
    data['data'] = []

    pklname = os.path.join(dataset, f"{width}x{height}px.pkl")

    print("Start Preprocess")

    for subdir in os.listdir(dataset):
        print(f"\t{subdir}")
        dir = os.path.join(dataset, subdir)

        for file in os.listdir(dir):
            if file[-3:] in {'jpg', 'png'}:
                print(f"\t\t{file}")
                path = os.path.join(dir, file)
                im = imread(path)
                im = resize(im, (width, height)) #[:,:,::-1]
                data['label'].append(subdir)
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
