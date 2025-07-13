# Get started

## Abstracts

* Input simple onnx file and inference sample tensor

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.26 or higher
* git-lfs
  * Refer [git-lfs](https://github.com/git-lfs/git-lfs?tab=readme-ov-file#getting-started) to install

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [ONNX Runtime GenAI](https://github.com/microsoft/onnxruntime-genai)
  * v0.8.2
  * MIT license

## Test Data

* [australia.jpg](https://github.com/microsoft/onnxruntime-genai/blob/main/test/test_models/images/australia.jpg)

## How to build?

First, go to [ONNXRuntimeGenAI/C](..).
Once time you built `ONNX Runtime GenAI`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ git submodule update --init --recursive .
$ git -C Phi-3.5-vision-instruct-onnx lfs pull
$ .\install\win\bin\Demo.exe .\Phi-3.5-vision-instruct-onnx\cpu_and_mobile\cpu-int4-rtn-block-32-acc-level-4 cpu
--------------------
Hello, Phi-3-Vision!
--------------------
C++ API
Creating config...  
Creating model...   
Creating multimodal processor...
Image Path (comma separated; leave empty if no image):
australia.jpg
Loading images...
Prompt:
How may cars?
Processing images and prompt...
Generating response...
 There is one car visible in the image.
````

#### Linux

````bash
$ git submodule update --init --recursive .
$ git -C Phi-3.5-vision-instruct-onnx lfs pull
$ LD_LIBRARY_PATH=install/linux/bin ./install/linux/bin/Demo ./Phi-3.5-vision-instruct-onnx/cpu_and_mobile/cpu-int4-rtn-block-32-acc-level-4 cpu
--------------------
Hello, Phi-3-Vision!
--------------------
C++ API
Creating config...
Creating model...
Creating multimodal processor...
Image Path (comma separated; leave empty if no image):
australia.jpg
Loading images...
Prompt: 
How many cars?
Processing images and prompt...
Generating response...
 There is one car visible in the image.
````

#### OSX

````bash
$ git submodule update --init --recursive .
$ git -C Phi-3.5-vision-instruct-onnx lfs pull
$ DYLD_LIBRARY_PATH=install/osx/bin ./install/osx/bin/Demo ./Phi-3.5-vision-instruct-onnx/cpu_and_mobile/cpu-int4-rtn-block-32-acc-level-4 cpu
````