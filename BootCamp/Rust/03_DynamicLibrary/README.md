# Create New Project (Dynamic Library)

## Abstracts

* Create new dynamic library
* This library is compatible with windows DOS

## How to start?

### Init

````cmd
$ cargo new sample --lib
     Created library `sample` package
````

### Edit config

Edit `cargo.toml`.

````diff
[package]
name = "sample"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

+ [lib]
+ name = "sample"
+ crate-type = ["dylib"]

[dependencies]
````

### Build

````cmd
$ cd sample
$ cargo build --release
   Compiling sample v0.1.0 (E:\Works\OpenSource\Demo\BootCamp\Rust\03_DynamicLibrary\sample)
    Finished release [optimized] target(s) in 0.63s
````

You will see `sample/target/release/sample.dll.lib` and `sample/target/release/sample.dll`.

But this sample does not expose any functions.
You can not use it!