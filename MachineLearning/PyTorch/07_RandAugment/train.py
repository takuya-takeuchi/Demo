import os
import argparse
import datetime

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms

from tensorboardX import SummaryWriter
from tqdm import tqdm
from collections import OrderedDict

import utils

from augmentations import RandAugment

program_name = "07_RandAugment"

# setup logger
logger = utils.get_logger("train")

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--epoch", required=True, type=int)
    parser.add_argument("--batchsize", required=True, type=int)
    parser.add_argument("--dataset-rootdir", required=True, type=str)
    parser.add_argument("--tensorboard-logdir", type=str)
    parser.add_argument("--randaugment", action="store_true")
    parser.add_argument("--n", required=False, help="parameter N (magnitude), from 0 to 14", type=int, default=0)
    parser.add_argument("--m", required=False, help="parameter M (magnitude), from 0 to 20", type=int, default=0)
    return parser.parse_args()

def train(args):
    epoch              = args.epoch
    batchsize          = args.batchsize
    dataset_rootdir    = args.dataset_rootdir
    tensorboard_logdir = args.tensorboard_logdir
    randaugment        = args.randaugment
    n                  = args.n
    m                  = args.m

    # check whether gpu is available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # setup transform
    train_transform = transforms.Compose([
        transforms.Resize([224, 224]),
        transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
    ])
    eval_transform = transforms.Compose([
        transforms.Resize([224, 224]),
        transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
    ])

    if randaugment:
        train_transform.transforms.insert(0, RandAugment(n, m))

    # setup data loader for train and eval
    train_dataset = torchvision.datasets.ImageFolder(os.path.join(dataset_rootdir, "train"),
                                                     transform=train_transform)
    eval_dataset = torchvision.datasets.ImageFolder(os.path.join(dataset_rootdir, "eval"),
                                                    transform=eval_transform)

    train_dataloader = torch.utils.data.DataLoader(train_dataset,
                                                   batch_size = batchsize,
                                                   shuffle = True)
    eval_dataloader = torch.utils.data.DataLoader(eval_dataset,
                                                  batch_size = batchsize)

    # get labels
    labels = utils.get_labels_from_dir(os.path.join(dataset_rootdir, "train"))

    # setup network, optimizer, scheduler and criterion
    model = torchvision.models.wide_resnet50_2(pretrained=True)
    features = model.fc.in_features
    model.fc = nn.Linear(features, len(labels))

    model.to(device)
    model.train()
    optimizer = optim.Adam(params=model.parameters(),
                           lr=1e-3)
    scheduler = optim.lr_scheduler.OneCycleLR(optimizer=optimizer,
                                              max_lr=1e-3,
                                              total_steps=len(train_dataloader))
    criterion = nn.CrossEntropyLoss()

    # tensorboard
    writer = None
    if tensorboard_logdir and os.path.exists(tensorboard_logdir):
        now = datetime.datetime.now()
        d = f"{now:%Y%m%d_%H%M%S}"
        logger.info(f"Setup Tensorboard Summary Writer: {tensorboard_logdir} for {program_name}_{d}")
        writer = SummaryWriter(log_dir=tensorboard_logdir, comment=f"{program_name}_{d}")

    logger.info("Start Training")

    best_accuracy = 0.0
    for e in range(epoch):
        accuracy = 0.0
        train_loss = 0.0

        model.train()
        with tqdm(train_dataloader) as pbar:
            pbar.set_description(f"[Epoch {e + 1}/{epoch}]")

            for images, labels in pbar:
                optimizer.zero_grad()
                
                images = images.to(device)
                labels = labels.to(device)

                out = model(images)
                loss = criterion(out, labels)

                loss.backward()
                optimizer.step()

                train_loss += loss.item()

                pbar.set_postfix(OrderedDict(Loss=train_loss / len(train_dataset)))

            scheduler.step()
        
        model.eval()
        with tqdm(eval_dataloader) as pbar:
            for images, labels in pbar:                
                images = images.to(device)
                labels = labels.to(device)

                out = model(images)

                preds = out.argmax(axis=1)

                accu = torch.sum(preds == labels).item()
                accuracy += accu

                pbar.set_postfix(OrderedDict(Accuracy=accuracy / len(eval_dataset)))

        accuracy = accuracy / len(eval_dataset)
        if writer:
            writer.add_scalar("loss", train_loss / len(train_dataset), e)
            writer.add_scalar("accuracy", accuracy, e)     
            writer.add_scalar("lr", scheduler.get_last_lr()[0], e)  

        if best_accuracy < accuracy:
            logger.info(f"Update Best Accuracy: from {best_accuracy} to {accuracy}")
            if randaugment:
                torch.save(model.state_dict(), f"trained_best_{n}_{m}.pth")
            else:
                torch.save(model.state_dict(), "trained_best.pth")
            best_accuracy = accuracy
    
    logger.info("Finished Training")
    logger.info(f"Best Accuracy: {best_accuracy}")

if __name__ == '__main__':
    # parse args
    args = get_args()
    epoch              = args.epoch
    batchsize          = args.batchsize
    dataset_rootdir    = args.dataset_rootdir
    tensorboard_logdir = args.tensorboard_logdir
    randaugment        = args.randaugment
    n                  = args.n
    m                  = args.m

    logger.info("Arguments")
    logger.info("               epoch: {}".format(epoch))
    logger.info("           batchsize: {}".format(batchsize))
    logger.info("     dataset_rootdir: {}".format(dataset_rootdir))
    logger.info("  tensorboard_logdir: {}".format(tensorboard_logdir))
    logger.info("         randaugment: {}".format(randaugment))
    logger.info("                   n: {}".format(n))
    logger.info("                   m: {}".format(m))

    train(args)
