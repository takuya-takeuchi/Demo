# Get started

## Abstracts

* Listing unnecessary `#include`

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or higher
* Python 3
  * Alias of python 3 shall be `python`

### Windows

* Visual Studio 2022
* Ninja
  * Ninja should be installed since Visual Studio 2017

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [include-what-you-use](https://github.com/include-what-you-use/include-what-you-use)
  * LLVM Release License

## How to build?

### GStreamer

Go to [IncludeWhatYouUse](..).

Once time you built `Include What You Use`, you need not to do again.

````shell
$ pwsh download-llvm.ps1
$ pwsh build.ps1 <Debug/Release>
````

## How to do?

This demo lists unnecessary `#include` when build.

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
Install the project...
-- Install configuration: "Release"
-- Up-to-date: /Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/install/osx/bin/Demo

/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should add these lines:

/Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should remove these lines:
- #include <fstream>  // lines 1-1

The full include-list for /Users/t-takeuchi/Work/OpenSource/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp:
#include <iostream>  // for char_traits, basic_ostream, cout, endl, operator<<
---
````