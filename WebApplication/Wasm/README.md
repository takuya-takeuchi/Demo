# Emscripten SDK (emsdk)

## Requirements

### Windows

* Python 3.6 or later

## How to use?

### Windows  

#### 1. Install emsdk

````bat
$ git submodule update --init --recursive .
$ cd emsdk
$ emsdk.bat install <emsdk_version>
````

For examples,

````bat
$ git submodule update --init --recursive .
$ cd emsdk
$ emsdk.bat install 3.1.36
````

After this, you can see emscripten was installed to `<emsdk root directory>/upstream/emscripten>`.

#### 2. Generate .emscripten

````bat
$ emsdk.bat activate latest
````

You can see `.emscripten` in `<emsdk root directory>`

#### 3. Check command

````bat
$ emcc --version
emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 3.1.36 (518d9fea335f7cd2b5771e43df76e0535c6df5dd)
Copyright (C) 2014 the Emscripten authors (see AUTHORS.txt)
This is free and open source software under the MIT license.
There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
````

#### 4. Option

You can use `emsdk` from anywhere.
For that, first you add `EM_CONFIG` environmental variable to specify location of `emsdk`.

For examples,

````bat
$ setx EM_CONFIG <emsdk root directory>\.emscripten
$ setx PATH "%PATH%;<emsdk root directory>\upstream\emscripten"
````

After this, copy `.emscripten` to where you want to invoke `emsdk`.

### Linux

````sh
$ git submodule update --init --recursive .
$ cd emsdk
$ ./emsdk install <emsdk_version>
````

For examples,

````bat
$ git submodule update --init --recursive .
$ cd emsdk
$ ./emsdk install 3.1.36
````

After this, you can see emscripten was installed to `<emsdk root directory>/upstream/emscripten>`.

#### 2. Generate .emscripten

````bat
$ ./emsdk activate latest
$ source ./emsdk_env.sh
````

#### 3. Check command

````bat
$ emcc --version
emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 3.1.36 (518d9fea335f7cd2b5771e43df76e0535c6df5dd)
Copyright (C) 2014 the Emscripten authors (see AUTHORS.txt)
This is free and open source software under the MIT license.
There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
````

### OSX

````sh
$ git submodule update --init --recursive .
$ cd emsdk
$ ./emsdk install <emsdk_version>
````

For examples,

````bat
$ git submodule update --init --recursive .
$ cd emsdk
$ ./emsdk install 3.1.36
````

After this, you can see emscripten was installed to `<emsdk root directory>/upstream/emscripten>`.

#### 2. Generate .emscripten

````bat
$ ./emsdk activate latest
$ source ./emsdk_env.sh
````

#### 3. Check command

````bat
$ emcc --version
emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 3.1.36 (518d9fea335f7cd2b5771e43df76e0535c6df5dd)
Copyright (C) 2014 the Emscripten authors (see AUTHORS.txt)
This is free and open source software under the MIT license.
There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
````

