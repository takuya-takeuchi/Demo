# Draw non-ascii text by using opnecv_contrib with FreeType

## Abstracts

* Draw non-ascii text like Japanese into cv::Mat by putText function

## Requirements

### Common

* Powershell 7 or later
* CMake 3.15.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License
* [opencv_contrib](https://github.com/opencv/opencv_contrib)
  * 4.7.0
  * Apache-2.0 License

## Test Data

* [NotoSansJP-VariableFont_wght.ttf](https://fonts.google.com/noto/specimen/Noto+Sans+JP)
  * SIL Open Font License 1.1

## How to build?

### OpenCV 4, FreeType and Harfbuzz

Go to [OpenCV](..).

````shell
$ pwsh build-freetype.ps1 <Debug/Release>
$ pwsh build-harfbuzz.ps1 <Debug/Release>
$ pwsh build-enable-freetype.ps1 <Debug/Release>
````

Once time you built `opencv4`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ .\install\win\bin\Demo.exe
````

<img src="./images/japanese.png" />