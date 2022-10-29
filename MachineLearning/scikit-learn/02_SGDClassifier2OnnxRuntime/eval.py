import argparse

import numpy as np
import onnxruntime as rt
from skimage.io import imread
from skimage.transform import resize

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=True)
    parser.add_argument("--image", type=str, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    return parser.parse_args()

def eval(args):
    model  = args.model
    image  = args.image
    width  = args.width
    height = args.height

    im = imread(image)
    im = resize(im, (width, height)) #[:,:,::-1]
    x = np.array(im) / 255

    # convert from [batch, width, height, channel] to [batch, width * height * channel]
    x = x.reshape(1, 3 * width * height)
    
    print("Start Evaluation")
    session = rt.InferenceSession(model)
    input_name = session.get_inputs()[0].name
    label_name = session.get_outputs()[0].name
    predict = session.run([label_name], {input_name: x.astype(np.float32)})
    print("Finished Evaluation")

    print(predict)

if __name__ == '__main__':
    # parse args
    args = get_args()
    image   = args.image
    model   = args.model
    width   = args.width
    height  = args.height

    print("Arguments")
    print("\t  image: {}".format(image))
    print("\t  model: {}".format(model))
    print("\t  width: {}".format(width))
    print("\t height: {}".format(height))

    eval(args)
