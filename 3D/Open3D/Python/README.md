# Open3D

## Requirements

### Common

* Python 3.7 or later

## Dependencies

* [open3d](https://github.com/isl-org/Open3D)
  * v0.18.0
  * MIT license

## How to build?

`numpy==2.0` may not work for open3d.
For example, visualize window does not show in windows.
See [Visualization always crashes (Win 11) #6903](https://github.com/isl-org/Open3D/issues/6903).

#### Windows

````bat
$ python3 -m venv .venv
$ .venv\Scripts\activate
$ python -m pip install pip --upgrade
$ python -m pip install open3d==0.18.0 matplotlib "numpy<=2.0.0"
````

#### Linux

````bat
$ python3 -m venv .venv
$ source .venv/bin/activate
$ python -m pip install pip --upgrade
$ python -m pip install open3d==0.18.0 matplotlib "numpy<=2.0.0"
````