# Quantization for ResNet50 on CPU

## Abstracts

* Do auantization for ResNet50 model
* This code is based on [onnxruntime-inference-examples/quantization/image_classification/cpu](https://github.com/microsoft/onnxruntime-inference-examples/tree/main/quantization/image_classification/cpu)

## Requirements

### Common

* Powershell 7 or later
* Python 3.8 - 3.11

## Dependencies

* [ONNX](https://github.com/onnx/onnx)
  * 1.16.1
  * Apache-2.0 license
* [ONNX Runtime](https://onnxruntime.ai/)
  * 1.16.3
  * MIT license

#### Miscellaneous

* [miniImageNet](https://lyy.mpi-inf.mpg.de/mtl/download)

## How to do?

### 1. Preprocess

````shell
$ python -m onnxruntime.quantization.preprocess --input resnetv2_50_Opset18.onnx  --output resnetv2_50_Opset18_preprocessed.onnx
````

#### 2. Quantize with calibrate

It takes a long time.
Download miniImageNet dataset and deploy it into [images](./images/).

````shell
$ python run.py --input_model resnetv2_50_Opset18_preprocessed.onnx --output_model resnetv2_50_Opset18_quant.onnx --calibrate_dataset ./images/train/
100%|██████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 38400/38400 [03:48<00:00, 168.22it/s]
Calibration and quantization start
Calibrated and quantized model saved. 2325.7759482860565 sec
benchmarking fp32 model...
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 10/10 [00:00<00:00, 27.78it/s]
Avg: 35.43ms
benchmarking int8 model...
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 10/10 [00:00<00:00, 29.12it/s] 
Avg: 34.00ms
````

#### 3. Benchmark

Run [benchmark.py](./benchmark.py) and [accuracy.py](./accuracy.py).
For examples,

````shell
$ python benchmark.py --input_model resnetv2_50_Opset18_quant.onnx
benchmarking model...
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 1000/1000 [00:37<00:00, 26.35it/s]
Avg: 37.79ms

$ python accuracy.py --input_model resnetv2_50_Opset18_quant.onnx  --dataset images\val
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 9600/9600 [07:30<00:00, 21.33it/s]
accuracy: 29.271 %
````

##### Speed

|CPU|Quantized (int8)|Original (float32)|
|---|---|---|
|Intel Core i7-8700 CPU @ 3.20GHz|37.79 ms|36.47 ms|
|Apple M2|20.78 ms|28.50 ms|

##### Accuracy

Use validation data of miniImageNet.

|CPU|Quantized (int8)|Original (float32)|
|---|---|---|
|Intel Core i7-8700 CPU @ 3.20GHz|29.271 %|35.000 %|
|Apple M2|29.135 %|35.000 %|