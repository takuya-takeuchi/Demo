# Missing Win32API

## Abstracts

* A certain Win32 APIs are not available between diffrent WindowsTargetPlatformMinVersion and WindowsTargetPlatformVersion

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

This example does not provide any useful functions.
You can see build errors when build.
For examples,

````bat
$ pwsh build.ps1 Release
  Building Custom Rule D:/Works/OpenSource/Demo/UWP/01_MissingAPI/CMakeLists.txt
  main.cpp
D:\Works\OpenSource\Demo\UWP\01_MissingAPI\main.cpp(36,9): error C3861: 'SetThreadGroupAffinity': 識別子が見つかりませんでした [D:\Works\OpenSource\Demo\UWP\01_MissingAPI\build\win\10.0.17134.0\program\Demo.vcxproj]
C:\Program Files (x86)\Windows Kits\10\bin\10.0.17134.0\XamlCompiler\Microsoft.Windows.UI.Xaml.Common.targets(449,5): error MSB4181: "CompileXaml" タスクから false が返されましたが、エラーがログに記録されませんでした。 [D:\Works\OpenSource\Demo\UWP\01_MissingAPI\build\win\10.0.17134.0\program\Demo.vcxproj]
````

and

````bat
$ pwsh build.ps1 Release
  Building Custom Rule D:/Works/OpenSource/Demo/UWP/01_MissingAPI/CMakeLists.txt
  main.cpp
  Demo.vcxproj -> D:\Works\OpenSource\Demo\UWP\01_MissingAPI\build\win\10.0.19041.0\program\Release\Demo.exe
C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Microsoft\VisualStudio\v17.0\AppxPackage\Microsoft.AppXPackage.Targets(2798,5): warning : パブリッシャー名 (CN=CMake) が署名証明書のサブジェクト: CN=CMake Test Cert と一致しません。パブリッシャー名を更新しています。 [D:\Works\OpenSource\Demo\UWP\01_MissingAPI\build\win\10.0.19041.0\program\Demo.vcxproj]
  Demo -> D:\Works\OpenSource\Demo\UWP\01_MissingAPI\build\win\10.0.19041.0\program\AppPackages\Demo\Demo_1.0.0.0_x64_Test\Demo_1.0.0.0_x64.msix
````
