import argparse
import math

import onnx
import onnxruntime

import torch
import torchvision
import torchvision.transforms as transforms

import numpy as np
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

    # add output nodes for intermediate layers
    model = onnx.load(pretrained)
    shape_info = onnx.shape_inference.infer_shapes(model)

    target_node_names = ["Conv_0", "Conv_3"]
    inter_layers = list()
    for node in model.graph.node:
        if node.name in target_node_names:
            inter_layers.append(node.output[0])

    value_info_protos = []
    for _, node in enumerate(shape_info.graph.value_info):
        if node.name in inter_layers:
            value_info_protos.append(node)

    assert len(value_info_protos) == len(inter_layers)
    model.graph.output.extend(value_info_protos)  #  in inference stage, these tensor will be added to output dict.
    onnx.checker.check_model(model)
    onnx.save(model, 'tmp.onnx')

    img = np.array(Image.open(image))
    inputs = img / 255
    # normalization
    # transform = transforms.Compose([
    #     transforms.ToTensor(),
    #     transforms.Normalize((0.5), (0.5))
    # ])
    inputs = inputs - 0.5
    inputs = inputs / 0.5

    # reshape
    # [28, 28] -> [1, 1, 28, 28] (B, C, H, W)
    shape = inputs.shape
    inputs = inputs.reshape(1, 1, shape[0], shape[1])
    
    # setup network
    session = onnxruntime.InferenceSession("tmp.onnx")

    logger.info("input:")
    for session_input in session.get_inputs():
        logger.info(f"\t{session_input.name}: {session_input.shape}")
    logger.info("output:")
    for session_output in session.get_outputs():
        logger.info(f"\t{session_output.name}: {session_output.shape}")

    logger.info("Start Visualization")

    outputs = ["22"]
    for name in inter_layers:
        outputs.append(name)
    preds, feature0, feature3 = session.run(outputs,
                        {"input.1": inputs.astype("float32")})

    def feature_to_img(feature):
        shape = feature.shape
        b = shape[0]
        c = shape[1]
        h = shape[2]
        w = shape[3]

        b = int(math.sqrt(b))
        
        img = torchvision.utils.make_grid(feature, nrow=b, normalize=True, pad_value=1)
        img = transforms.functional.to_pil_image(img)
        new_w = w * b
        new_h = int(new_w * img.height / img.width)
        img = img.resize((new_w, new_h))

        return img

    for f, name in zip([feature0, feature3], target_node_names):
        f = f.transpose(1, 0, 2, 3)
        img = feature_to_img(torch.from_numpy(f))
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
