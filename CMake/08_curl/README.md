# curl from CMake

## Abstracts

* Build and link curl by cmake
  * static build
* Minimal example program

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio 2019

### Ubuntu

* g++
* libssl-dev
  * via `apt` command

### OSX

* Xcode
* openssl
  * via `brew` command

## Dependencies

* [curl](https://github.com/curl/curl)
  * 7.87.0
  * MIT/X Derivative License

## How to usage?

````shell
$ pwsh build-curl.ps1  <Debug/Release>
$ pwsh build.ps1  <Debug/Release>
https://phet-dev.colorado.edu/html/build-an-atom/0.0.0-3/simple-text-only-test-page.html
<!DOCTYPE html>
<!-- Build an Atom using Easel for the Scene Graph Library -->
<!--[if lt IE 7]>
<html class="no-js lt-ie9 lt-ie8 lt-ie7"> <![endif]-->
<!--[if IE 7]>
<html class="no-js lt-ie9 lt-ie8"> <![endif]-->
<!--[if IE 8]>
<html class="no-js lt-ie9"> <![endif]-->
<!--[if gt IE 8]><!-->
<html class="no-js"> <!--<![endif]-->
<head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge,chrome=1">
    <meta name="description" content="">
    <meta name="viewport"
          content="width=device-width, height=device-height, initial-scale=1.0, maximum-scale=1.0, user-scalable=no"/>
    <meta name="apple-mobile-web-app-capable" content="yes">
    <meta name="apple-mobile-web-app-status-bar-style" content="white">

    <title>Simple Text Only Test Page</title>
</head>

<body>

<h1>Simple Test Page</h1>

<p>This page tests that simple text can be rendered using the basic HTML5 boiler plate outline.</p>

</body>
</html>
````