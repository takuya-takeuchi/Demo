# Microsoft Style

## Abstracts

* Apply Google C++ coding style

## Requirements

### Common

* Powershell 7 or later

## Dependencies

* [LLVM](https://releases.llvm.org)
  * 18.1.8
  * Apache-2.0 with LLVM-exception license

## How to use?

````shell
$ pwsh Run.ps1
````

#### Before

````cpp
#include <iostream>
void main(){
std::cout<<"Hello world!!"<<std::endl;
}
````

#### After

````cpp
#include <iostream>
void main()
{
    std::cout << "Hello world!!" << std::endl;
}
````