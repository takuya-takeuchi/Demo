# Inverse of a matrix

## Abstracts

* Calculate the inverse of a matrix and compare diffrence between `np.linalg.inv` and `nc::linalg::inv`

## Requirements

* Powershell 7 or later
* CMake 3.14.0 or later
* C++ compiler support C++ 17

## Windows

* Visual Studio 2022

## Linux

* GCC

## OSX

* XCode

## Dependencies

* [NumCpp](https://github.com/dpilger26/NumCpp)
  * 2.11.0
  * MIT License

## How to usage?

### cpp

Load [conv.dat](./conv.dat) and calculate the inverse of a matrix.
And write result to `conv_inv_cpp.dat`.

````cmd
$ pwsh Build.ps1 <Release/Debug>
````

If enable `boost`, you have to change `-D NUMCPP_NO_USE_BOOST="ON"` to `-D NUMCPP_NO_USE_BOOST="OFF"` in [Build.ps1](./Build.ps1).

### python

Load [conv.dat](./conv.dat) and calculate the inverse of a matrix.
And write result to `conv_inv_py.dat`.

````cmd
$ python3 main.py
````

### Test

Compare between `conv_inv_py.dat` and `conv_inv_cpp.dat`.
Calculate diffrence of each element.

### Windows

````cmd
$ python3 compare.py
tolerance: 1e-05
    total: 1.848698167681152e-12
    count: 0
````

### Linux

````cmd
$ python3 compare.py
tolerance: 1e-05
    total: 1.790190142731741e-12
    count: 0
````