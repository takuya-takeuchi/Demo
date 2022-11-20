# Convert to ONNX

## Abstracts

* Visualize feature maps of LeNet model for MNIST

## Requirements

* Python 3.6 or later
* PyTorch 1.10.2

Please check [requirements.txt](./requirements.txt)

## How to usage?

````cmd
$ python -m pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu113
````

### Train

````cmd
$ python train.py --epoch 100 --batchsize 512 --tensorboard-logdir ..\00_Tensorboard\logs
````

### Convert

````cmd
$ python convert.py --pretrained trained.pth
2022-11-20 13:02:08,686 [INFO] Arguments
2022-11-20 13:02:08,687 [INFO]      pretrained: trained.pth
LeNet(
  (conv1): Conv2d(1, 6, kernel_size=(5, 5), stride=(1, 1), padding=(2, 2))
  (pool1): MaxPool2d(kernel_size=2, stride=2, padding=0, dilation=1, ceil_mode=False)
  (conv2): Conv2d(6, 16, kernel_size=(5, 5), stride=(1, 1))
  (pool2): MaxPool2d(kernel_size=2, stride=2, padding=0, dilation=1, ceil_mode=False)
  (flatten): Flatten(start_dim=1, end_dim=-1)
  (fc1): Linear(in_features=400, out_features=120, bias=True)
  (fc2): Linear(in_features=120, out_features=84, bias=True)
  (fc3): Linear(in_features=84, out_features=10, bias=True)
)
2022-11-20 13:02:08,841 [INFO] Start Conversion
graph(%input.1 : Float(1, 1, 28, 28, strides=[784, 784, 28, 1], requires_grad=0, device=cuda:0),
      %conv1.weight : Float(6, 1, 5, 5, strides=[25, 25, 5, 1], requires_grad=1, device=cuda:0),
      %conv1.bias : Float(6, strides=[1], requires_grad=1, device=cuda:0),
      %conv2.weight : Float(16, 6, 5, 5, strides=[150, 25, 5, 1], requires_grad=1, device=cuda:0),
      %conv2.bias : Float(16, strides=[1], requires_grad=1, device=cuda:0),
      %fc1.weight : Float(120, 400, strides=[400, 1], requires_grad=1, device=cuda:0),
      %fc1.bias : Float(120, strides=[1], requires_grad=1, device=cuda:0),
      %fc2.weight : Float(84, 120, strides=[120, 1], requires_grad=1, device=cuda:0),
      %fc2.bias : Float(84, strides=[1], requires_grad=1, device=cuda:0),
      %fc3.weight : Float(10, 84, strides=[84, 1], requires_grad=1, device=cuda:0),
      %fc3.bias : Float(10, strides=[1], requires_grad=1, device=cuda:0)):
  %11 : Float(1, 6, 28, 28, strides=[4704, 784, 28, 1], requires_grad=1, device=cuda:0) = onnx::Conv[dilations=[1, 1], group=1, kernel_shape=[5, 5], pads=[2, 2, 2, 2], strides=[1, 1]](%input.1, %conv1.weight, %conv1.bias) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\modules\conv.py:443:0
  %12 : Float(1, 6, 28, 28, strides=[4704, 784, 28, 1], requires_grad=1, device=cuda:0) = onnx::Sigmoid(%11) # E:\Works\OpenSource\Demo\MachineLearning\PyTorch\05_ToOnnx\models\lenet.py:19:0
  %13 : Float(1, 6, 14, 14, strides=[1176, 196, 14, 1], requires_grad=1, device=cuda:0) = onnx::MaxPool[kernel_shape=[2, 2], pads=[0, 0, 0, 0], strides=[2, 2]](%12) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\functional.py:719:0
  %14 : Float(1, 16, 10, 10, strides=[1600, 100, 10, 1], requires_grad=1, device=cuda:0) = onnx::Conv[dilations=[1, 1], group=1, kernel_shape=[5, 5], pads=[0, 0, 0, 0], strides=[1, 1]](%13, %conv2.weight, %conv2.bias) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\modules\conv.py:443:0
  %15 : Float(1, 16, 10, 10, strides=[1600, 100, 10, 1], requires_grad=1, device=cuda:0) = onnx::Sigmoid(%14) # E:\Works\OpenSource\Demo\MachineLearning\PyTorch\05_ToOnnx\models\lenet.py:21:0
  %16 : Float(1, 16, 5, 5, strides=[400, 25, 5, 1], requires_grad=1, device=cuda:0) = onnx::MaxPool[kernel_shape=[2, 2], pads=[0, 0, 0, 0], strides=[2, 2]](%15) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\functional.py:719:0
  %17 : Float(1, 400, strides=[400, 1], requires_grad=1, device=cuda:0) = onnx::Flatten[axis=1](%16) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\modules\flatten.py:42:0
  %18 : Float(1, 120, strides=[120, 1], requires_grad=1, device=cuda:0) = onnx::Gemm[alpha=1., beta=1., transB=1](%17, %fc1.weight, %fc1.bias) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\functional.py:1848:0
  %19 : Float(1, 120, strides=[120, 1], requires_grad=1, device=cuda:0) = onnx::Sigmoid(%18) # E:\Works\OpenSource\Demo\MachineLearning\PyTorch\05_ToOnnx\models\lenet.py:24:0
  %20 : Float(1, 84, strides=[84, 1], requires_grad=1, device=cuda:0) = onnx::Gemm[alpha=1., beta=1., transB=1](%19, %fc2.weight, %fc2.bias) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\functional.py:1848:0
  %21 : Float(1, 84, strides=[84, 1], requires_grad=1, device=cuda:0) = onnx::Sigmoid(%20) # E:\Works\OpenSource\Demo\MachineLearning\PyTorch\05_ToOnnx\models\lenet.py:25:0
  %22 : Float(1, 10, strides=[10, 1], requires_grad=1, device=cuda:0) = onnx::Gemm[alpha=1., beta=1., transB=1](%21, %fc3.weight, %fc3.bias) # D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\torch\nn\functional.py:1848:0
  return (%22)

2022-11-20 13:02:11,965 [INFO] Finished Conversion
````

#### conv1

<img src="images/conv1.png?raw=true" title="conv1"/>

#### conv2

<img src="images/conv2.png?raw=true" title="conv2"/>