import argparse

import torchvision.transforms as transforms

import numpy as np
np.set_printoptions(threshold=np.inf, precision=4, floatmode='fixed',linewidth=200)

import cv2

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, type=str)
    return parser.parse_args()

def train(args):
    image = args.image

    # setup transform
    transform = transforms.Compose([
        transforms.ToTensor(),
        # transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
    ])

    img = cv2.imread(image)
    tensor = transform(img).numpy()

    print("tensor[0]")
    print(tensor[0])
    print("tensor[1]")
    print(tensor[1])
    print("tensor[2]")
    print(tensor[2])

if __name__ == '__main__':
    # parse args
    args = get_args()
    image = args.image

    print("Arguments")
    print("\timage: {}".format(image))

    train(args)
