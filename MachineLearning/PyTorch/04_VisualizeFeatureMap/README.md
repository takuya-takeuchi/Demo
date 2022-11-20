# Visualize Feature maps

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

### Visualize

````cmd
$ python visualize.py --pretrained trained.pth --image input.png
2022-11-20 12:44:43,260 [INFO] Arguments
2022-11-20 12:44:43,261 [INFO]      pretrained: trained.pth
2022-11-20 12:44:43,261 [INFO]           image: input.png
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
(['x', 'conv1', 'sigmoid', 'pool1', 'conv2', 'sigmoid_1', 'pool2', 'flatten', 'fc1', 'sigmoid_2', 'fc2', 'sigmoid_3', 'fc3'], ['x', 'conv1', 'sigmoid', 'pool1', 'conv2', 'sigmoid_1', 'pool2', 'flatten', 'fc1', 'sigmoid_2', 'fc2', 'sigmoid_3', 'fc3'])
2022-11-20 12:44:43,434 [INFO] Start Visualization
2022-11-20 12:44:45,432 [INFO] Finished Visualization
````

#### conv1

<img src="images/conv1.png?raw=true" title="conv1"/>

#### conv2

<img src="images/conv2.png?raw=true" title="conv2"/>