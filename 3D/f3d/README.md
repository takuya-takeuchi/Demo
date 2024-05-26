# f3d

## Requirements

### Common

* Powershell 7 or later
* CMake

### Windows

* Visual Studio

## Dependencies

* [f3d](https://github.com/f3d-app/f3d)
  * v2.4.0
  * BSD-3-Clause license
* [GLFW](https://github.com/glfw/glfw)
  * 3.4
  * Zlib license
* [VTK](https://github.com/Kitware/VTK)
  * v9.3.0
  * BSD-3-Clause license

## How to build?

````bat
$ pwsh build-VTK.ps1 <Debug/Release>
$ pwsh build-glfw.ps1 <Debug/Release>
$ pwsh build.ps1 <Debug/Release>
````