# clang-format

## Abstracts

* Format C++ code by clang-format

## Dependencies

* [LLVM](https://releases.llvm.org)
  * 18.1.8
  * Apache-2.0 with LLVM-exception license

## How to install?

````bash
$ winget install llvm --version 18.1.8
````

After this, add `C:\Program Files\LLVM\bin` to environmental variable `Path`.

### Ubuntu

````sh
$ sudo apt install clang-format
````

### OSX

````sh
$ brew install clang-format
````

## How to use?

````sh
$ clang-format --style=file:"/path/to/.clang-format" <source-file> > <dest-file>
````

You can use inplace mode.

````sh
$ clang-format --style=file:"/path/to/.clang-format" -i <source-file>
````

And you can use `*` to search wild card.

````sh
$ clang-format --style=file:"/path/to/.clang-format" -i *.cpp
````