# Expose functions to C/C++

## Abstracts

* Create new dynamic library and expose function to C/C++
* Invoke Rust functions from C/C++

## Requirements

* Powershell 7 or later
* CMake 3.9.0 or later

## How to start?

### Init

Check [03_DynamicLibrary](../03_DynamicLibrary) to initialize package to build dynamic library.

### Coding

Edit `lib.rs`

````rs
#[no_mangle]
pub extern "C" fn add(a: i32, b: i32) -> i32
{
	return a + b;
}
````

### Build

````cmd
$ cd sample
$ cargo build --release
   Compiling sample v0.1.0 (E:\Works\OpenSource\Demo\BootCamp\Rust\04_ExposeFunctionToCandCPP\sample)
    Finished release [optimized] target(s) in 0.75s
````

You will see `sample/target/release/sample.dll.lib` and `sample/target/release/sample.dll`.

### Run program with dll createad by Rust

[cc](./cc) provides simple c console program.
This program is build by CMake via Powershell.

````cmd
$ cd cc
$ pwsh Build.ps1
-- Selecting Windows SDK version 10.0.22000.0 to target Windows 10.0.19044.
-- The C compiler identification is MSVC 19.34.31933.0
-- The CXX compiler identification is MSVC 19.34.31933.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.34.31933/bin/Hostx64/x64/cl.exe - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.34.31933/bin/Hostx64/x64/cl.exe - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: E:/Works/OpenSource/Demo/BootCamp/Rust/04_ExposeFunctionToCandCPP/cc/build_win
MSBuild version 17.4.0+18d5aef85 for .NET Framework
  Checking Build System
  Building Custom Rule E:/Works/OpenSource/Demo/BootCamp/Rust/04_ExposeFunctionToCandCPP/cc/CMakeLists.txt
  main.cpp
  cc.vcxproj -> E:\Works\OpenSource\Demo\BootCamp\Rust\04_ExposeFunctionToCandCPP\cc\build_win\Release\cc.exe
  Building Custom Rule E:/Works/OpenSource/Demo/BootCamp/Rust/04_ExposeFunctionToCandCPP/cc/CMakeLists.txt
  -- Install configuration: "Release"
  -- Installing: E:/Works/OpenSource/Demo/BootCamp/Rust/04_ExposeFunctionToCandCPP/cc/win/bin/cc.exe
  CMake Warning (dev) at cmake_install.cmake:48 (file):
    Syntax error in cmake code at

      E:/Works/OpenSource/Demo/BootCamp/Rust/04_ExposeFunctionToCandCPP/cc/build_win/cmake_install.cmake:48

    when parsing string

      E:\Works\OpenSource\Demo\BootCamp\Rust\04_ExposeFunctionToCandCPP\sample\target\release/sample.dll

    Invalid escape sequence \W

    Policy CMP0010 is not set: Bad variable reference syntax is an error.  Run
    "cmake --help-policy CMP0010" for policy details.  Use the cmake_policy
    command to set the policy and suppress this warning.
  This warning is for project developers.  Use -Wno-dev to suppress it.

  -- Installing: E:/Works/OpenSource/Demo/BootCamp/Rust/04_ExposeFunctionToCandCPP/cc/win/bin/sample.dll
10 + 2 = 12
````