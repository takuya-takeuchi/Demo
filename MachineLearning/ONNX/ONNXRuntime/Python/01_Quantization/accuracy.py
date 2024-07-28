import argparse
import json
import os
import pathlib
import numpy as np
import onnxruntime
import time
from tqdm import tqdm
import time

from resnet50_data_reader import _preprocess_image
import numpy as np

def softmax(x):
    e_x = np.exp(x - np.max(x))
    return e_x / e_x.sum()

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_model", required=True, help="input model")
    parser.add_argument("--dataset", default="./images/train", help="data set")
    args = parser.parse_args()
    return args

def main():
    args = get_args()
    input_model_path = args.input_model
    dataset_path = args.dataset

    session = onnxruntime.InferenceSession(input_model_path)
    (_, channel, height, width) = session.get_inputs()[0].shape
    input_name = session.get_inputs()[0].name

    input_data = np.zeros((1, channel, height, width), np.float32)
    # Warming up
    _ = session.run([], {input_name: input_data})

    json_open = open('classes.json', 'r')
    json_load = json.load(json_open)
    classes = []
    for key in json_load.keys():
        classes.append(key)

    p = pathlib.Path(dataset_path)
    image_names = [f for f in p.glob("**/*.jpg")]

    correct = 0
    wrong = 0
    for image_name in tqdm(image_names):
        input_data = _preprocess_image(image_name, height, width)
        logits = session.run([], {input_name: input_data})
        predicts = softmax(logits)
        cls = np.argmax(predicts)
        class_name = os.path.basename(os.path.dirname(image_name))
        gt = classes.index(class_name)
        # print(f"predict: {cls}, gt: {gt}")
        if cls == classes.index(class_name):
            correct += 1
        else:
            wrong += 1

    accuracy = correct * 100 / len(image_names)
    print("accuracy: {:.3f} %".format(accuracy))

if __name__ == "__main__":
    main()
