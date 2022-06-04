import os
import argparse

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms

from tqdm import tqdm
from collections import OrderedDict

from models.net import Net
import utils

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--epoch", required=True, type=int)
    parser.add_argument("--batchsize", required=True, type=int)
    return parser.parse_args()

def train(args):
    epoch           = args.epoch
    batchsize       = args.batchsize

    # check whether gpu is available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # setup transform
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    # setup data loader for train
    trainset = torchvision.datasets.CIFAR10(root='./data',
                                            train=True,
                                            download=True,
                                            transform=transform)
    trainloader = torch.utils.data.DataLoader(trainset,
                                              batch_size=batchsize,
                                              shuffle=True,
                                              num_workers=2)

    # setup network, optimizer, scheduler and criterion
    model = Net()
    model.to(device)
    model.train()
    optimizer = optim.SGD(params=model.parameters(),
                          lr=1e-3)
    scheduler = optim.lr_scheduler.OneCycleLR(optimizer=optimizer,
                                              max_lr=1e-3,
                                              total_steps=len(trainloader))
    criterion = nn.CrossEntropyLoss()

    print("Start Training")

    for e in range(epoch):
        accuracy, train_loss = 0.0, 0.0

        with tqdm(trainloader) as pbar:
            pbar.set_description(f"[Epoch {e + 1}/{epoch}]")

            for images, labels in pbar:
                optimizer.zero_grad()
                
                images = images.to(device)
                labels = labels.to(device)

                out = model(images)
                loss = criterion(out, labels)

                loss.backward()
                optimizer.step()

                preds = out.argmax(axis=1)

                train_loss += loss.item()
                accuracy += torch.sum(preds == labels).item() / len(labels)

                pbar.set_postfix(OrderedDict(Loss=loss.item(),
                                             Accuracy=torch.sum(preds == labels).item() / len(labels)))

            print(f"Epoch: {e + 1}")
            print(f"Loss: {train_loss / len(trainloader)}")
            print(f"Accuracy: {accuracy / len(trainloader)}")

            scheduler.step()
    
    print("Finished Training")

    torch.save(model.state_dict(), "trained.pth")

if __name__ == '__main__':
    # parse args
    args = get_args()
    epoch           = args.epoch
    batchsize       = args.batchsize

    print("Arguments")
    print("          epoch: {}".format(epoch))
    print("      batchsize: {}".format(batchsize))

    train(args)
