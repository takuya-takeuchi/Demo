import os
import argparse
import pickle

import onnxmltools
from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    return parser.parse_args()

def eval(args):
    model  = args.model
    width  = args.width
    height = args.height

    # im = imread(image)
    # im = resize(im, (width, height)) #[:,:,::-1]
    # x = np.array(im) / 255

    # # convert from [batch, width, height, channel] to [batch, width * height * channel]
    # x = x.reshape(1, 3 * width * height)
    
    saved_model = pickle.load(open(model, 'rb'))

    print("Start Conversion")
    initial_types = [('float_input', FloatTensorType([1, height * width * 3]))]
    onnx_model = convert_sklearn(saved_model,
                                 initial_types=initial_types,
                                 target_opset=12)
    print("Finished Conversion")

    directory = os.path.dirname(model)
    filename = os.path.splitext(os.path.basename(model))[0]

    output = os.path.join(directory, f'{filename}.onnx')
    onnx_model_path = output
    onnxmltools.utils.save_model(onnx_model, onnx_model_path)

if __name__ == '__main__':
    # parse args
    args = get_args()
    model   = args.model
    width   = args.width
    height  = args.height

    print("Arguments")
    print("\t  model: {}".format(model))
    print("\t  width: {}".format(width))
    print("\t height: {}".format(height))

    eval(args)
