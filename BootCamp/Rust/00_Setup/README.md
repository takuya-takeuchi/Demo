# Setup Rust development Environmental

## Abstracts

* Setup Rust Development Environmental

## Windows

### Install C++ compiler

* Install `Microsoft C++ Build Tools` or `Visual Studio`
  * Requires only C++ build tool

### Install Rust

* Download installer from [Install Rust](https://www.rust-lang.org/tools/install) and kick it!
* Intaller shows commad prompt
  * Basically you can only input `1`

````cmd
The Cargo home directory is located at:

  C:\Users\TAKUYA\.cargo

This can be modified with the CARGO_HOME environment variable.

The cargo, rustc, rustup and other commands will be added to
Cargo's bin directory, located at:

  C:\Users\TAKUYA\.cargo\bin

This path will then be added to your PATH environment variable by
modifying the HKEY_CURRENT_USER/Environment/PATH registry key.

You can uninstall at any time with rustup self uninstall and
these changes will be reverted.

Current installation options:


   default host triple: x86_64-pc-windows-msvc
     default toolchain: stable (default)
               profile: default
  modify PATH variable: yes

1) Proceed with installation (default)
2) Customize installation
3) Cancel installation
>1
````

### Check

````cmd
$ cargo version
cargo 1.66.0 (d65d197ad 2022-11-15)
````