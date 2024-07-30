import pathlib
import numpy as np
import onnxruntime
from onnxruntime.quantization import CalibrationDataReader
from PIL import Image
from tqdm import tqdm

def _normalize(image, mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225]):
    mean = np.array(mean, dtype=np.float32)
    std = np.array(std, dtype=np.float32)
    image = np.array(image, dtype=np.float32) / 255.0
    normalized_image = (image - mean) / std    
    return normalized_image

def _preprocess_image(image_filepath: str, height: int, width: int):
    pillow_img = Image.new("RGB", (width, height))
    pillow_img.paste(Image.open(image_filepath).resize((width, height)))
    input_data = _normalize(pillow_img)
    nhwc_data = np.expand_dims(input_data, axis=0)
    nchw_data = nhwc_data.transpose(0, 3, 1, 2)  # ONNX Runtime standard
        
    return nchw_data

def _preprocess_images(images_folder: str, height: int, width: int, size_limit=0):
    p = pathlib.Path(images_folder)
    image_names = [f for f in p.glob("**/*.jpg")]
    if size_limit > 0 and len(image_names) >= size_limit:
        batch_filenames = [image_names[i] for i in range(size_limit)]
    else:
        batch_filenames = image_names
    unconcatenated_batch_data = []

    for image_name in tqdm(batch_filenames):
        unconcatenated_batch_data.append(_preprocess_image(image_name, height, width))
    batch_data = np.concatenate(
        np.expand_dims(unconcatenated_batch_data, axis=0), axis=0
    )
    return batch_data


class ResNet50DataReader(CalibrationDataReader):
    def __init__(self, calibration_image_folder: str, model_path: str):
        self.enum_data = None

        # Use inference session to get input shape.
        session = onnxruntime.InferenceSession(model_path, None)
        (_, _, height, width) = session.get_inputs()[0].shape

        # Convert image to input data
        self.nhwc_data_list = _preprocess_images(
            calibration_image_folder, height, width, size_limit=0
        )
        self.input_name = session.get_inputs()[0].name
        self.datasize = len(self.nhwc_data_list)

    def get_next(self):
        if self.enum_data is None:
            self.enum_data = iter(
                [{self.input_name: nhwc_data} for nhwc_data in self.nhwc_data_list]
            )
        return next(self.enum_data, None)

    def rewind(self):
        self.enum_data = None
