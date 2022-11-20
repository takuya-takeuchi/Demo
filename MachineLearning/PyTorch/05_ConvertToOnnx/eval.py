import argparse

import onnxruntime

import numpy as np
from PIL import Image

from models.lenet import LeNet
import utils

# setup logger
logger = utils.get_logger("eval")

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pretrained", required=True, type=str)
    parser.add_argument("--image", required=True, type=str)
    return parser.parse_args()

def train(args):
    pretrained = args.pretrained
    image      = args.image

    # setup network
    session = onnxruntime.InferenceSession(pretrained)

    logger.info("input:")
    for session_input in session.get_inputs():
        logger.info(f"\t{session_input.name}: {session_input.shape}")
    logger.info("output:")
    for session_output in session.get_outputs():
        logger.info(f"\t{session_output.name}: {session_output.shape}")

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

    logger.info("Start Evaluation")
    
    preds = session.run(["22"],
                        {"input.1": inputs.astype("float32")})

    def softmax(x):
        u = np.sum(np.exp(x))
        return np.exp(x) / u

    preds = softmax(preds[0])
    index = preds.argmax()    

    logger.info("Finished Evaluation")

    logger.info(f"Result: {index}: [{preds[0][index]}]")

if __name__ == '__main__':
    # parse args
    args = get_args()
    pretrained = args.pretrained
    image      = args.image

    logger.info("Arguments")
    logger.info("     pretrained: {}".format(pretrained))
    logger.info("          image: {}".format(image))

    train(args)
