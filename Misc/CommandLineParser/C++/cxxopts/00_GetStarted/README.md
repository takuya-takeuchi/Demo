# Get started

## Abstracts

* How to parsae command line arguments by `cxxopts`

## Requirements

### Common

* Powershell 7 or later
* CMake 3.20.0 or later

### Windows

* Visual Studio

### Linux

* g++

### OSX

* Xcode

## Dependencies

* [cxxopts](https://github.com/jarro2783/cxxopts)
  * v3.3.1
  * MIT license

## How to build?

### cxxopts

Go to [cxxopts](..).
Once time you built `cxxopts`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\server.exe
````

#### Linux

````bash
$ ./install/linux/bin/server
````

#### OSX

````bash
$ ./install/osx/Release/bin/Demo --name test -w 90.0 -a 20 -c Red -f "file1" -f "file2" 
        Name: test
         Age: 20
      Weight: 90
       Color: 0
       Files: file1, file2
Verbose mode: OFF
````