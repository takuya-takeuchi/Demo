import os
import argparse
import datetime
from logging import config, getLogger
import yaml

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms

from tensorboardX import SummaryWriter
from tqdm import tqdm
from collections import OrderedDict

from models.lenet import LeNet
import utils

program_name = "02_LeNet"

# setup logger
logger = utils.get_logger()

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--epoch", required=True, type=int)
    parser.add_argument("--batchsize", required=True, type=int)
    parser.add_argument("--tensorboard-logdir", type=str)
    return parser.parse_args()

def train(args):
    epoch              = args.epoch
    batchsize          = args.batchsize
    tensorboard_logdir = args.tensorboard_logdir

    # check whether gpu is available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # setup transform
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5), (0.5))
    ])

    # setup data loader for train
    trainset = torchvision.datasets.MNIST(root='../data',
                                          train=True,
                                          download=True,
                                          transform=transform)
    trainloader = torch.utils.data.DataLoader(trainset,
                                              batch_size=batchsize,
                                              shuffle=True)

    # setup network, optimizer, scheduler and criterion
    model = LeNet()
    model.to(device)
    model.train()
    optimizer = optim.Adam(params=model.parameters(),
                          lr=1e-3)
    scheduler = optim.lr_scheduler.OneCycleLR(optimizer=optimizer,
                                              max_lr=1e-3,
                                              total_steps=len(trainloader))
    criterion = nn.CrossEntropyLoss()

    # tensorboard
    writer = None
    if os.path.exists(tensorboard_logdir):
        now = datetime.datetime.now()
        d = f"{now:%Y%m%d_%H%M%S}"
        logger.info(f"Setup Tensorboard Summary Writer: {tensorboard_logdir} for {program_name}_{d}")
        writer = SummaryWriter(log_dir=tensorboard_logdir, comment=f"{program_name}_{d}")

    logger.info("Start Training")

    for e in range(epoch):
        accuracy = 0.0
        train_loss = 0.0

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
                accu = torch.sum(preds == labels).item()
                accuracy += accu

                pbar.set_postfix(OrderedDict(Loss=train_loss / len(trainset),
                                             Accuracy=accuracy / len(trainset)))

            scheduler.step()

        if writer:
            writer.add_scalar("loss", train_loss / len(trainset), e)
            writer.add_scalar("accuracy", accuracy / len(trainset), e)     
            writer.add_scalar("lr", scheduler.get_last_lr()[0], e)  
    
    logger.info("Finished Training")

    torch.save(model.state_dict(), "trained.pth")

if __name__ == '__main__':
    # parse args
    args = get_args()
    epoch              = args.epoch
    batchsize          = args.batchsize
    tensorboard_logdir = args.tensorboard_logdir

    logger.info("Arguments")
    logger.info("               epoch: {}".format(epoch))
    logger.info("           batchsize: {}".format(batchsize))
    logger.info("  tensorboard_logdir: {}".format(tensorboard_logdir))

    train(args)
