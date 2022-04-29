# ArUco Marker

## Abstacts

* How to detect ArUco marker

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

You must input camera intrinsic parameter from [05_CalibrateCamera](../05_CalibrateCamera).

````cmd
$ dotnet run -c Release --dictionary Dict7X7_50 --parameter calibration.yaml --size 0.15 --output
````

|Dictionary Name|
|---|
|Dict4X4_50|
|Dict4X4_100|
|Dict4X4_250|
|Dict4X4_1000|
|Dict5X5_50|
|Dict5X5_100|
|Dict5X5_250|
|Dict5X5_1000|
|Dict6X6_50|
|Dict6X6_100|
|Dict6X6_250|
|Dict6X6_1000|
|Dict7X7_50|
|Dict7X7_100|
|Dict7X7_250|
|Dict7X7_1000|
|DictArucoOriginal|
|DictAprilTag_16h5|
|DictAprilTag_25h9|
|DictAprilTag_36h10|
|DictAprilTag_36h11|

## Result

[![captured](./images/captured.jpg "captured")](./images/captured.jpg)

## Misc

You can download ArUci marker sheet from [ArUco marker sheet generator!](https://fodi.github.io/arucosheetgen/) and [ArUco markers generator!](https://chev.me/arucogen/).

### ArUco marker sheet generator!

[DICT_7X7_50_100_250_1000_0_20_20.pdf](DICT_7X7_50_100_250_1000_0_20_20.pdf) is generated from

|Key|Value|
|---|---|
|Dictionary|7x7 (50, 100, 250, 1000)|
|First marker ID|0|
|Last marker ID|20|
|Maeker size, mm|20|

### ArUco markers generator!

[DICT_7X7_50_100_250_1000_0_150.pdf](DICT_7X7_50_100_250_1000_0_150.pdf) is generated from

|Key|Value|
|---|---|
|Dictionary|7x7 (50, 100, 250, 1000)|
|Marker ID|0|
|Maeker size, mm|150|