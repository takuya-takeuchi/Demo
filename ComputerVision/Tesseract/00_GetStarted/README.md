# Get started

## Abstracts

* Input image and recognize text and bounding boxes

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

* [tesseract](https://github.com/tesseract-ocr/tesseract)
  * 5.4.0
  * Apache-2.0 license
* [tessdata](https://github.com/tesseract-ocr/tessdata)
  * 4.1.0
  * Apache-2.0 license
* [leptonica](https://github.com/danbloomberg/leptonica)
  * 1.84.1
  * BSD 2-clause license

## How to build?

### Tesseract

Go to [Tesseract](..).

````shell
$ pwsh build.ps1 <Debug/Release>
````

Once time you built `Tesseract`, you need not to do again.

## How to test?

### Windows

````bat
$ pwsh build.ps1 <Debug/Release>
$ set TESSDATA_PREFIX=..\tessdata
$ .\install\win\bin\Demo.exe testdata\logo.png
Symbol: T Confidence: 98.9253 BoundingBox: [0, 162, 39, 216]
Symbol: e Confidence: 98.876 BoundingBox: [44, 175, 77, 217]
Symbol: s Confidence: 98.8815 BoundingBox: [84, 175, 113, 217]
Symbol: s Confidence: 98.7692 BoundingBox: [119, 175, 148, 217]
Symbol: e Confidence: 98.8635 BoundingBox: [156, 175, 189, 217]
Symbol: r Confidence: 99.0289 BoundingBox: [199, 175, 221, 216]
Symbol: a Confidence: 98.8477 BoundingBox: [225, 175, 256, 217]
Symbol: c Confidence: 98.8872 BoundingBox: [266, 175, 295, 217]
Symbol: t Confidence: 98.9076 BoundingBox: [299, 166, 321, 217]
Symbol: O Confidence: 98.5532 BoundingBox: [347, 161, 396, 217]
Symbol: C Confidence: 98.985 BoundingBox: [405, 161, 445, 217]
Symbol: R Confidence: 98.9922 BoundingBox: [455, 162, 490, 216]
````

### OSX

````bat
$ pwsh build.ps1 <Debug/Release>
$ export TESSDATA_PREFIX=../tessdata
$ ./install/osx/bin/Demo testdata/logo.png
Error in pixReadMemTiff: function not present
Error in pixReadMem: tiff: no pix returned
Error in pixaGenerateFontFromString: pix not made
Error in bmfCreate: font pixa not made
Symbol: T Confidence: 98.9253 BoundingBox: [0, 162, 39, 216]
Symbol: e Confidence: 98.876 BoundingBox: [44, 175, 77, 217]
Symbol: s Confidence: 98.8815 BoundingBox: [84, 175, 113, 217]
Symbol: s Confidence: 98.7692 BoundingBox: [119, 175, 148, 217]
Symbol: e Confidence: 98.8635 BoundingBox: [156, 175, 189, 217]
Symbol: r Confidence: 99.0289 BoundingBox: [199, 175, 221, 216]
Symbol: a Confidence: 98.8477 BoundingBox: [225, 175, 256, 217]
Symbol: c Confidence: 98.8872 BoundingBox: [266, 175, 295, 217]
Symbol: t Confidence: 98.9076 BoundingBox: [299, 166, 321, 217]
Symbol: O Confidence: 98.5532 BoundingBox: [347, 161, 396, 217]
Symbol: C Confidence: 98.985 BoundingBox: [405, 161, 445, 217]
Symbol: R Confidence: 98.9922 BoundingBox: [455, 162, 490, 216]
````

### Linux

````bat
$ pwsh build.ps1 <Debug/Release>
$ export TESSDATA_PREFIX=../tessdata
$ ./install/linux/bin/Demo testdata/logo.png
Error in pixReadMemTiff: function not present
Error in pixReadMem: tiff: no pix returned
Error in pixaGenerateFontFromString: pix not made
Error in bmfCreate: font pixa not made
Symbol: T Confidence: 98.9035 BoundingBox: [0, 162, 39, 216]
Symbol: e Confidence: 98.9339 BoundingBox: [44, 175, 77, 217]
Symbol: s Confidence: 98.8969 BoundingBox: [84, 175, 113, 217]
Symbol: s Confidence: 98.736 BoundingBox: [119, 175, 148, 217]
Symbol: e Confidence: 98.6103 BoundingBox: [156, 175, 189, 217]
Symbol: r Confidence: 99.0139 BoundingBox: [199, 175, 221, 216]
Symbol: a Confidence: 98.6043 BoundingBox: [225, 175, 256, 217]
Symbol: c Confidence: 98.917 BoundingBox: [266, 175, 295, 217]
Symbol: t Confidence: 98.9681 BoundingBox: [299, 166, 321, 217]
Symbol: O Confidence: 98.7526 BoundingBox: [347, 161, 396, 217]
Symbol: C Confidence: 98.9665 BoundingBox: [405, 161, 445, 217]
Symbol: R Confidence: 98.9826 BoundingBox: [455, 162, 490, 216]
````