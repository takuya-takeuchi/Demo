# Invoke template function via template

## Abstracts

* Example of usage of `template` keyword: When calling a template function of a member through a template, this keyword is added to indicate that it is a template function.

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.0 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* N/A

## How to use?

This example is demonstration to confirm whether build get succeeded or failed.
When build in OSX or Linux, you get error message after change `s.template hello<10>();` to `s.hello<10>();`.

````cpp
template<typename T>
void test()
{
    T s;
#ifdef _WINDOWS
    // or s.template hello<10>();
    s.hello<10>();
#else
    // gcc says 'error: invalid operands of types ‘<unresolved overloaded function type>’ and ‘int’ to binary ‘operator<’'
    // xcode says 'error: missing 'template' keyword prior to dependent template name 'hello''
    // s.hello<10>();
    s.template hello<10>();
#endif
}
````

````shell
$ pwsh build.ps1  <Debug/Release>
````