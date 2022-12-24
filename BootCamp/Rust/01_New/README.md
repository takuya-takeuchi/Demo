# Create New Project

## Abstracts

* Create new package (a.k.a project)
* Create executable binary

## How to start?

### Init

````cmd
$ cd sample
$ cargo init
````

#### :warning: Warning

`cargo init` command check current directory name. 
Because directory name is used as project name.

If directory name is invalid, `cargo` command output message like this.

````cmd
error: the name `01_New` cannot be used as a package name, the name cannot start with a digit
If you need a package name to not match the directory name, consider using --name flag.
If you need a binary with the name "01_New", use a valid package name, and set the binary name to be different from the package. This can be done by setting the binary filename to `src/bin/01_New.rs` or change the name in Cargo.toml with:

    [[bin]]
    name = "01_New"
    path = "src/main.rs"
````

### Build

`cargo build` command output binary as debug.

````cmd
$ cd sample
$ cargo build
   Compiling sample v0.1.0 (E:\Works\OpenSource\Demo\BootCamp\Rust\01_New\sample)
    Finished dev [unoptimized + debuginfo] target(s) in 0.58s
````

You can output as release.

````cmd
$ cd sample
$ cargo build --release
   Compiling sample v0.1.0 (E:\Works\OpenSource\Demo\BootCamp\Rust\01_New\sample)
    Finished release [optimized] target(s) in 0.45s
````

### Run

You can only kick *.exe.

````cmd
$ target\release\sample.exe
Hello, world!
````