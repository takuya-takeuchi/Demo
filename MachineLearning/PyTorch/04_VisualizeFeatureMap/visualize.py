import argparse
import math

import torch
import torchvision
import torchvision.transforms as transforms

from PIL import Image

from models.lenet import LeNet
import utils

# setup logger
logger = utils.get_logger("visualize")

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pretrained", required=True, type=str)
    parser.add_argument("--image", required=True, type=str)
    return parser.parse_args()

def train(args):
    pretrained = args.pretrained
    image      = args.image

    # check whether gpu is available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # setup transform
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5), (0.5))
    ])

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
    names = torchvision.models.feature_extraction.get_graph_node_names(model)
    print(names)

    extractor = torchvision.models.feature_extraction.create_feature_extractor(
        model, ["conv1", "conv2"]
    )

    def feature_to_img(feature):
        shape = feature.shape
        c = shape[0]
        h = shape[1]
        w = shape[2]

        c = int(math.sqrt(c))

        feature = feature.unsqueeze(1)
        img = torchvision.utils.make_grid(feature.cpu(), nrow=c, normalize=True, pad_value=1)
        img = transforms.functional.to_pil_image(img)
        new_w = w * c
        new_h = int(new_w * img.height / img.width)
        img = img.resize((new_w, new_h))

        return img

    logger.info("Start Visualization")

    img = Image.open(image)
    inputs = transform(img)
    inputs = inputs.unsqueeze(0).to(device)
    features = extractor(inputs)

    for name, x in features.items():
        img = feature_to_img(x[0])
        img.save(f"{name}.png")

    logger.info("Finished Visualization")

if __name__ == '__main__':
    # parse args
    args = get_args()
    pretrained = args.pretrained
    image      = args.image

    logger.info("Arguments")
    logger.info("     pretrained: {}".format(pretrained))
    logger.info("          image: {}".format(image))

    train(args)
