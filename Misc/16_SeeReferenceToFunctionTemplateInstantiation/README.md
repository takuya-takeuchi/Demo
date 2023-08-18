# See reference to function template instantiation

## Abstracts

* How to resolve `message : see reference to function template instantiation`

## Requirements

### Windows

* Visual Studio
* CMake 3.0 or later

## How to resolve?

### Japanese message

````bat
$ pwsh Build.ps1

E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\main.cpp(9,34): warning C4101: 'ex': ローカル変数は 1 度も使われていません。 [E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\build\win\Demo.vcxproj]
E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\main.cpp(18,12): message : コンパイル対象の関数 テンプレート インスタンス化 'int run_loop<char>(int,const char_type *[])' のリファレンスを確認してください [E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\build\win\Demo.vcxproj]
          with
          [
              char_type=char
          ]
````

### English message

````bat
$ pwsh Build.ps1

E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\main.cpp(9,34): warning C4100: 'argv': unreferenced formal parameter [E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\build\win\Demo.vcxproj]
E:\Demo2\Misc\16_SeeReferenceToFunctionTemplateInstantiation\main.cpp(18,12): message : see reference to function template instantiation 'int run_loop(int,const char_type *[])'
          with
          [
              char_type=char
          ]
````

This compiler messages does not stand for its own, it accompanies another message.
To remove this annoying message,

* Resolve warning `C4101`

````diff
int run_loop(int argc, const char_type* argv[])
{
    try
    {
    }
    catch (const std::exception& ex)
    {
+        std::cout << ex.what() << std::endl;
    }

    return 0;
}
````

or

````diff
int run_loop(int argc, const char_type* argv[])
{
    try
    {
    }
-   catch (const std::exception& ex)
+   catch (const std::exception&)
    {
    }

    return 0;
}
````

* Suppress warning `C4101`

````diff
cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

+if (MSVC)
+    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w44101")
+endif()

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)
````