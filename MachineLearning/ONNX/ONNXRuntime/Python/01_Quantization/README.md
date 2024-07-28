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

## How to do?

### 1. Preprocess

````shell
$ python -m onnxruntime.quantization.preprocess --input resnetv2_50_Opset18.onnx  --output resnetv2_50_Opset18_preprocessed.onnx
````

#### 2. Quantize with calibrate

It takes a long time.

````shell
$ E:\Works\OpenSource\Demo\MachineLearning\ONNX\ONNXRuntime\Python\01_Quantization>python run.py --input_model resnetv2_50_Opset18_preprocessed.onnx --output_model resnetv2_50_Opset18_quant.onnx --calibrate_dataset ./images/train/
100%|██████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 38400/38400 [03:48<00:00, 168.22it/s]
Calibrated and quantized start
Calibrated and quantized model saved. 2325.7759482860565 sec
benchmarking fp32 model...
  0%|                                                                                                                                                          | 0/10 [00:00<?, ?it/s]33.21ms
38.46ms
36.35ms
 30%|███████████████████████████████████████████▊                                                                                                      | 3/10 [00:00<00:00, 27.52it/s]35.44ms
36.08ms
34.35ms
 60%|███████████████████████████████████████████████████████████████████████████████████████▌                                                          | 6/10 [00:00<00:00, 27.67it/s]36.25ms
34.60ms
35.37ms
 90%|███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████▍              | 9/10 [00:00<00:00, 27.72it/s]34.21ms
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 10/10 [00:00<00:00, 27.78it/s]
Avg: 35.43ms
benchmarking int8 model...
  0%|                                                                                                                                                          | 0/10 [00:00<?, ?it/s]37.46ms
32.92ms
33.42ms
 30%|███████████████████████████████████████████▊                                                                                                      | 3/10 [00:00<00:00, 28.72it/s3 
4.03ms
34.70ms
33.35ms
 60%|███████████████████████████████████████████████████████████████████████████████████████▌                                                          | 6/10 [00:00<00:00, 28.96it/s]33.34ms
33.90ms
33.81ms
 90%|███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████▍              | 9/10 [00:00<00:00, 29.03it/s]33.06ms
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 10/10 [00:00<00:00, 29.12it/s] 
Avg: 34.00ms
````

#### 3. Benchmark

Run [benchmark.py](./benchmark.py).
For examples,

````shell
$ python benchmark.py --input_model resnetv2_50_Opset18_quant.onnx
benchmarking model...
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 1000/1000 [00:37<00:00, 26.35it/s]
Avg: 37.79ms
````

|CPU|Quantized (int8)|Original (float32)|
|---|---|---|
|Intel Core i7-8700 CPU @ 3.20GHz|37.79 ms|36.47 ms|
|Apple M2|20.78 ms|28.50 ms|