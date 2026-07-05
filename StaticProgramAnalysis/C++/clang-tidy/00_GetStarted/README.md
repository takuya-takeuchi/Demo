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
404 warnings generated.
E:\Works\OpenSource\Demo\StaticProgramAnalysis\C++\clang-tidy\00_GetStarted\main.cpp:4:5: warning: use a trailing return type for this function [modernize-use-trailing-return-type]
    4 | int main() {
      | ~~~ ^
      | auto       -> int
E:\Works\OpenSource\Demo\StaticProgramAnalysis\C++\clang-tidy\00_GetStarted\main.cpp:5:16: warning: use nullptr [modernize-use-nullptr]
    5 |     int* ptr = 0; 
      |                ^
      |                nullptr
E:\Works\OpenSource\Demo\StaticProgramAnalysis\C++\clang-tidy\00_GetStarted\main.cpp:8:17: warning: C-style casts are discouraged; use static_cast [google-readability-casting,modernize-avoid-c-style-cast]
    8 |     int intPi = (int)pi; 
      |                 ^~~~~  
      |                 static_cast<int>( )
E:\Works\OpenSource\Demo\StaticProgramAnalysis\C++\clang-tidy\00_GetStarted\main.cpp:12:5: warning: use range-based for loop instead [modernize-loop-convert]
   12 |     for (size_t i = 0; i < numbers.size(); ++i)
      |     ^   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      |         (int number : numbers)
   13 |         std::cout << numbers[i] << " ";
      |                      ~~~~~~~~~~
      |                      number
E:\Works\OpenSource\Demo\StaticProgramAnalysis\C++\clang-tidy\00_GetStarted\main.cpp:12:48: warning: statement should be inside braces [google-readability-braces-around-statements]
   12 |     for (size_t i = 0; i < numbers.size(); ++i)
      |                                                ^
      |                                                 {
   13 |         std::cout << numbers[i] << " ";
      |                                        
Suppressed 398 warnings (396 in non-user code, 2 with check filters).
Use -header-filter=.* or leave it as default to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
````

#### Linux

````shell
$ pwsh build.ps1 <Debug/Release>
145 warnings generated.
/data/work/oss/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:4:5: warning: use a trailing return type for this function [modernize-use-trailing-return-type]
    4 | int main()
      | ~~~ ^     
      | auto       -> int
/data/work/oss/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:6:16: warning: use nullptr [modernize-use-nullptr]
    6 |     int* ptr = 0;
      |                ^
      |                nullptr
/data/work/oss/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:9:17: warning: C-style casts are discouraged; use static_cast [google-readability-casting,modernize-avoid-c-style-cast]
    9 |     int intPi = (int)pi;
      |                 ^~~~~  
      |                 static_cast<int>( )
/data/work/oss/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:13:5: warning: use range-based for loop instead [modernize-loop-convert]
   13 |     for (size_t i = 0; i < numbers.size(); ++i)
      |     ^   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      |         (int number : numbers)
   14 |         std::cout << numbers[i] << " ";
      |                      ~~~~~~~~~~
      |                      number
/data/work/oss/Demo/StaticProgramAnalysis/C++/clang-tidy/00_GetStarted/main.cpp:13:48: warning: statement should be inside braces [google-readability-braces-around-statements]
   13 |     for (size_t i = 0; i < numbers.size(); ++i)
      |                                                ^
      |                                                 {
   14 |         std::cout << numbers[i] << " ";
      |                                        
Suppressed 139 warnings (139 in non-user code).
Use -header-filter=.* or leave it as default to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
````

#### OSX

````shell
$ pwsh build.ps1 <Debug/Release>
590 warnings generated.
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
Suppressed 584 warnings (584 in non-user code).
Use -header-filter=.* or leave it as default to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
````