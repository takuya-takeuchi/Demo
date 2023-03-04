# cpplint

## Abstracts

* Analysis C++ source code

## Dependencies

* [cpplint](https://github.com/cpplint/cpplint)
  * 1.6.1
  * 3-Clause BSD License

## How to install?

### Windows

````sh
$ python -m venv .venv
$ .venv\Scripts\activate.bat
$ python -m pip install cpplint
````

### Others

````sh
$ python -m venv .venv
$ source .venv/bin/activate
$ python -m pip install cpplint
````

## How to use?

````sh
$ cpplint [OPTIONS] files
````

For examples, cpplint analysis this code `sample.cpp`.

````cpp
#include <iostream>

void main()
{
    std::cout << "Hello world!!" << std::endl;
}
````

````sh
$ cpplint sample.cpp 
sample.cpp:0:  No copyright message found.  You should have a line: "Copyright [year] <Copyright Owner>"  [legal/copyright] [5]
sample.cpp:4:  { should almost always be at the end of the previous line  [whitespace/braces] [4]
sample.cpp:6:  Could not find a newline character at the end of the file.  [whitespace/ending_newline] [5]
Done processing sample.cpp
Total errors found: 3
````