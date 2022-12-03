# Stream Raw Image of WebCam

## Abstracts

* Retrieve stream raw image rather than RGB from webcam
  * Disable/Enable converting to RGB
  * We tested on Microsoft LifeCam Studio

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

## How to usage?

### Disable Convert to RGB 

````cmd
$ dotnet run -c Release --project Demo\Demo.csproj -- -o output
2022-12-03 15:53:05.0826 [INFO ]                ConvertRgb: False 
2022-12-03 15:53:05.1160 [INFO ]                    Output: output 
2022-12-03 15:53:05.4593 [INFO ] Succeeded to change CAP_PROP_CONVERT_RGB from 1 to 0 
2022-12-03 15:53:05.4593 [INFO ] CAP_PROP_MODE: 1 
2022-12-03 15:53:05.4593 [INFO ] CAP_PROP_FOURCC: 844715353 (YUY2)
2022-12-03 15:53:06.0963 [INFO ] Width: 4147200, Height: 1, Channels: 1, Type: CV_8UC1, Total: 4147200 
2022-12-03 15:53:06.1326 [INFO ] Width: 4147200, Height: 1, Channels: 1, Type: CV_8UC1, Total: 4147200 
2022-12-03 15:53:06.1549 [INFO ] Width: 4147200, Height: 1, Channels: 1, Type: CV_8UC1, Total: 4147200 
````

### Enable Convert to RGB 

````cmd
$ dotnet run -c Release --project Demo\Demo.csproj -- -o output -c
2022-12-03 16:05:54.2535 [INFO ]                ConvertRgb: True 
2022-12-03 16:05:54.2904 [INFO ]                    Output: output 
2022-12-03 16:05:54.5913 [INFO ] CAP_PROP_MODE: 1 
2022-12-03 16:05:54.5913 [INFO ] CAP_PROP_FOURCC: 22 () 
2022-12-03 16:05:55.2562 [INFO ] Width: 1920, Height: 1080, Channels: 3, Type: CV_8UC3, Total: 2073600 
2022-12-03 16:05:55.2894 [INFO ] Width: 1920, Height: 1080, Channels: 3, Type: CV_8UC3, Total: 2073600 
2022-12-03 16:05:55.3097 [INFO ] Width: 1920, Height: 1080, Channels: 3, Type: CV_8UC3, Total: 2073600 
````