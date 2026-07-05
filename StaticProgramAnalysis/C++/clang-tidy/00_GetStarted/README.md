# Get started

## Abstracts

* Check source code

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or higher

### Windows

* Visual Studio 2022
* Ninja
  * Ninja should be installed since Visual Studio 2017

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [LLVM](https://releases.llvm.org)
  * Apache-2.0 with LLVM-exception license

## How to build?

### GStreamer

Go to [clang-tidy](..).

Once time you download `LLVM`, you need not to do again.

````shell
$ pwsh download-llvm.ps1
````

## How to do?

This demo checks source code after build.

#### Windows

````bat
$ pwsh build.ps1 <Debug/Release>
[0/1] Install the project...

E:/Works/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should add these lines:
#include <__msvc_ostream.hpp>  // for basic_ostream, endl, operator<<

E:/Works/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should remove these lines:
- #include <fstream>  // lines 1-1

The full include-list for E:/Works/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp:
#include <__msvc_ostream.hpp>  // for basic_ostream, endl, operator<<
#include <iostream>            // for char_traits, cout
---
````

#### Linux

````shell
$ pwsh build.ps1 <Debug/Release>
-- Install configuration: "Release"
-- Up-to-date: /data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/install/linux/bin/Demo

/data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should add these lines:

/data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should remove these lines:
- #include <fstream>  // lines 1-1

The full include-list for /data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp:
#include <iostream>  // for char_traits, basic_ostream, cout, endl, operator<<
---
````

#### OSX

````shell
$ pwsh build.ps1 <Debug/Release>
585 warnings generated.
/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:4:5: warning: use a trailing return type for this function[modernize-use-trailing-return-type]
    4 | int main() {
      | ~~~ ^
      | auto       -> int
/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:5:16: warning: use nullptr [modernize-use-nullptr]
    5 |     int* ptr = 0; 
      |                ^
      |                nullptr
/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:8:17: warning: C-style casts are discouraged; use static_cast [google-readability-casting,modernize-avoid-c-style-cast]
    8 |     int intPi = (int)pi; 
      |                 ^~~~~  
      |                 static_cast<int>( )
/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:12:5: warning: use range-based for loop instead [modernize-loop-convert]
   12 |     for (size_t i = 0; i < numbers.size(); ++i)
      |     ^   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      |         (int number : numbers)
   13 |         std::cout << numbers[i] << " ";
      |                      ~~~~~~~~~~
      |                      number
/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:12:48: warning: statement should be inside braces [google-readability-braces-around-statements]
   12 |     for (size_t i = 0; i < numbers.size(); ++i)
      |                                                ^
      |                                                 {
   13 |         std::cout << numbers[i] << " ";
      |                                        
Suppressed 579 warnings (579 in non-user code).
Use -header-filter=.* or leave it as default to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
````