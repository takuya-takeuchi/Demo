# Calibrate Camera

## Abstacts

* How to calibrate camera
* Get intrinsic parameter

## Requirements

* Visual Studio 2022
* .NET 6.0

## Dependencies

* [OpenCVSharp](https://github.com/shimat/opencvsharp)
  * Apache License 2.0
* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
* [CommandLineParser](https://github.com/commandlineparser/commandline)
  * MIT License
* [Fluent Validation](https://github.com/FluentValidation/FluentValidation)
  * Apache License 2.0

## How to usage?

````cmd
$ dotnet run -c Release --horizontal 10 --vertical 7 --size 2.5 --count 40 --output
````

## Result

This program outputs calibration.yaml.

````yaml
%YAML:1.0
---
Count: 40
ChessSize: 2.5000000000000000e+00
PatternRow: 7
PatternColumn: 10
RMS: 4.0276109697102624e+00
CameraIntrinsicMatrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1., 0., 5.0812033903546955e-30, 0., 1.,
       2.6269863510665807e-30, 0., 0., 1. ]
DistortionCoefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 4.6844699607907627e-21, 3.3044584353649073e-15,
       6.8826085680269663e-24, 1.3492452503084928e-23,
       -2.2444162298396161e-21 ]
````

[![captured](./images/captured.jpg "captured")](./images/captured.jpg)

## Misc

You can download chess pattern from [chesspattern_7x10.pdf](http://opencv.jp/sample/pics/chesspattern_7x10.pdf).