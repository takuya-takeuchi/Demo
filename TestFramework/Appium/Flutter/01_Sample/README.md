# Sample test

## Requirements

* Python

## How to use?

#### 1. bulild flutter app

|Target|Command|
|---|---|
|iPhone|`flutter build ios --profile`|
|iPhone Simulator|`flutter build ios --debug --simulator`|

#### 2. Run test

##### Python

At first, you should create virtual env.

````sh
$ cd python
$ python3 -m venv .venv
$ source .venv/bin/activate
$ python -m pip install -r requirement.txt
````

Then,

|Target|Command|
|---|---|
|iPhone|`python iphone-device.py`|
|iPhone Simulator|`python iphone-simulator.py`|


For examle or iPhone

````sh
$ % python example-simulator.py 
You have pushed the button this many times:
1
````

And you will see `screenshot-simulator.png` in current directory.

<img src="./images/python-screenshot-simulator.png" height="350" />