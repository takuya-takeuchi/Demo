import os
import argparse

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms

from tqdm import tqdm
from typing import OrderedDict

from models.net import Net
import utils

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, type=str)
    return parser.parse_args()

def train(args):
    pretrained = args.pretrained
    label      = args.label

    # setup transform
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    # setup data loader for test
    testset = torchvision.datasets.CIFAR10(root='./data',
                                           train=False,
                                           download=True,
                                           transform=transform)
    testloader = torch.utils.data.DataLoader(testset,
                                             batch_size=1,
                                             shuffle=False,
                                             num_workers=2)

    # setup network
    model = Net()
    model.load_state_dict(torch.load(pretrained))
    model.eval()

    print("Start Evaluation")

    accuracy = 0
    with tqdm(testloader) as pbar:
        for images, labels in pbar:
            out = model(images)
            preds = out.argmax(axis=1)

            accuracy += torch.sum(preds == labels).item() / len(labels)
    
    print("Finished Evaluation")

    print("Results")
    print(f"Accuracy: {accuracy / len(testloader)}")

if __name__ == '__main__':
    # parse args
    args = get_args()
    pretrained = args.pretrained
    label      = args.label

    print("Arguments")
    print("     pretrained: {}".format(pretrained))
    print("          label: {}".format(label))

    train(args)
