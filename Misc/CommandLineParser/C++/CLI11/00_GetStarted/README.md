# Get started

## Abstracts

* How to parsae command line arguments by `CLI11`

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

* [CLI11](https://github.com/cliutils/cli11)
  * 2.27.2
  * BSD-3-Clause license

## How to build?

### cxxopts

Go to [CLI11](..).
Once time you built `CLI11`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\Release\bin\Demo.exe --file test1.exe
Working on file: test1.exe, direct count: 1, opt count: 1

$ .\install\win\Release\bin\Demo.exe --file test1.exe --file test1.exe --file test2.exe
--file: At most 1 required but received 2
Run with --help for more information.
````

#### Linux

````bash
$ ./install/linux/bin/Demo --file test1.exe
Working on file: test1.exe, direct count: 1, opt count: 1

$ ./install/linux/bin/Demo --file test1.exe --file test2.exe
--file: At most 1 required but received 2
Run with --help for more information.
````

#### OSX

````bash
$ ./install/osx/Release/bin/Demo --file test1.exe
Working on file: test1.exe, direct count: 1, opt count: 1

$ ./install/osx/Release/bin/Demo --file test1.exe --file test2.exe
--file: At most 1 required but received 2
Run with --help for more information.
````