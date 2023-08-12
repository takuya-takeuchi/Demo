# Sample test

## Requirements

* Python

## How to use?

#### 1. bulild flutter app

|Target|Command|
|---|---|
|iPhone|`flutter build ios --profile`|
|iPhone Simulator|`flutter build ios --debug --simulator`|
|Android|flutter build apk --debug|

#### 2. Start Appium

For Windows,

````bat
$ npx appium
````

For Linux or OSX,

````sh
$ appium
````

If you saw `EADDRINUSE: address already in use 0.0.0.0:4723`, you have to change listen port number of appium.

````bat
$ npx appium -p 4725
````

#### 3. Run test

##### Precheck

You shall check proper device or simulator names.

###### Android

````sh
$ adb devices
List of devices attached
HQ618G0E28      device
emulator-5554   device
````

###### iOS

You can use `xcrun simctl list devices` or `idb_companion`.

##### Python

At first, you should create virtual env.

For Windows,

````bat
$ cd python
$ python3 -m venv .venv
$ venv\Scripts\activate
$ python -m pip install -r requirement.txt
````

For Linux or OSX,

````sh
$ cd python
$ python3 -m venv .venv
$ source .venv/bin/activate
$ python -m pip install -r requirement.txt
````

Then, you must update python scripts.
For examples, [iphone-simulator.py](./iphone-simulator.py)

````python
driver = Remote('http://localhost:4723', dict(
    platformName='iOS',
    automationName='flutter',
    platformVersion='16.0',
    deviceName='iPhone 14 Pro',
    app='{}/../demo/build/ios/Debug-iphonesimulator/Runner.app'.format(
      os.path.dirname(os.path.realpath(__file__)))
))
````

`platformVersion` and `deviceName` should be updated.

After modfied scripts, you can run test scripts.

|Target|Command|
|---|---|
|iPhone|`python iphone-device.py`|
|iPhone Simulator|`python iphone-simulator.py`|
|Android|`python android-device.py`|
|Android Emulator|`python android-emulator.py`|

For examle or iPhone

````sh
$ % python example-simulator.py 
You have pushed the button this many times:
1
````

And you will see `screenshot-simulator.png` in current directory.

<img src="./images/python-screenshot-simulator.png" height="350" />