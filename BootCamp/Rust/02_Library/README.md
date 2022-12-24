# Create New Project (Static Library)

## Abstracts

* Create new library
* This library is only for Rust
  * Binary is not compatible for C/C++!!

## How to start?

### Init

````cmd
$ cd sample
$ cargo init --lib
````

or

````cmd
$ cargo new sample --lib
     Created library `sample` package
````

### Build

````cmd
$ cd sample
$ cargo build --release
   Compiling sample v0.1.0 (E:\Works\OpenSource\Demo\BootCamp\Rust\02_Library\sample)
    Finished release [optimized] target(s) in 0.24s
````

You will see `sample/target/release/libsample.d` and `sample/target/release/libsample.rlib`.

* libsample.d is Makefile-compatible dependency lists
* libsample.rlib is static library
  * crate type is lib