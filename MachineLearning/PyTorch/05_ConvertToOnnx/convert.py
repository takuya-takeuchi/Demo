import argparse

import torch

from models.lenet import LeNet
import utils

# setup logger
logger = utils.get_logger("convert")

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pretrained", required=True, type=str)
    return parser.parse_args()

def train(args):
    pretrained = args.pretrained

    # check whether gpu is available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # setup network
    model = LeNet()
    model.load_state_dict(torch.load(pretrained))
    model.to(device)
    model.eval()

    print(model)
    # LeNet(
    #     (conv1): Conv2d(1, 6, kernel_size=(5, 5), stride=(1, 1), padding=(2, 2))
    #     (pool1): MaxPool2d(kernel_size=2, stride=2, padding=0, dilation=1, ceil_mode=False)
    #     (conv2): Conv2d(6, 16, kernel_size=(5, 5), stride=(1, 1))
    #     (pool2): MaxPool2d(kernel_size=2, stride=2, padding=0, dilation=1, ceil_mode=False)
    #     (flatten): Flatten(start_dim=1, end_dim=-1)
    #     (fc1): Linear(in_features=400, out_features=120, bias=True)
    #     (fc2): Linear(in_features=120, out_features=84, bias=True)
    #     (fc3): Linear(in_features=84, out_features=10, bias=True)
    # )

    logger.info("Start Conversion")

    batch = 1
    channel = 1
    height = 28
    width = 28
    dummy = torch.randn((batch, channel, height, width))
    torch.onnx.export(model, dummy.to(device), "lenet.onnx", verbose=True)

    logger.info("Finished Conversion")

if __name__ == '__main__':
    # parse args
    args = get_args()
    pretrained = args.pretrained

    logger.info("Arguments")
    logger.info("     pretrained: {}".format(pretrained))

    train(args)
