#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration,

   [Parameter(
   Mandatory=$False,
   Position = 2
   )][switch]
   $Static
)

$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "onnxruntime"
$emSDKVersion = "3.1.57"
$sharedType = $Static ? "static" : "shared"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $sharedType
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $sharedType | `
              Join-Path -ChildPath $Configuration
$buildLog = Join-Path $buildDir build.log

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $target

git submodule update --init --recursive .

if ($global:IsWindows)
{
    # * If you use webnn, you can not specify --disable_rtti.
    # * --minimal_build can reduce artifact size but onnx model files should be convert to ort model format.
    # * If you define EM_CONFIG envrironmental valur, it may occur something to wrong.
    # Do not use 'Visual Studio 17 2022` as cmake generator!!
    # $env:EM_CONFIG=""
    # chcp 65001
    $sharedTypeArg = $Static ? "--build_wasm_static_lib" : "--build_wasm"
    $webNNArg = $Static ? "--use_webnn" : "" # WebNN is only available for WebAssembly build.
    python tools/ci_build/build.py --config ${Configuration} `
                                   --cmake_generator Ninja `
                                   $sharedTypeArg `
                                   --emsdk_version ${emSDKVersion} `
                                   --enable_wasm_simd `
                                   --enable_wasm_threads `
                                   $webNNArg `
                                   --use_jsep `
                                   --disable_wasm_exception_catching `
                                   --parallel `
                                   --build_dir ${buildDir} `
                                   --skip_tests `
                                   --skip_onnx_tests `
                                   --cmake_extra_defines CMAKE_INSTALL_PREFIX="${installDir}" 2>&1 | Tee-Object -FilePath "${buildLog}" | ForEach-Object { $_ }

    # on windows, install is not executed so we have to kick install command!!
    $buildDir = Join-Path $buildDir ${Configuration}
    Push-Location $buildDir | Out-Null
    cmake --install .
    Pop-Location

    if ($Static -eq $False)
    {
        New-Item -Type Directory "${installDir}/bin" -Force | Out-Null
        Copy-Item "${buildDir}/${Configuration}/*.js" "${installDir}/bin" -Force -Recurse | Out-Null
        Copy-Item "${buildDir}/${Configuration}/*.wasm" "${installDir}/bin" -Force -Recurse | Out-Null
    }

    # npm install -g typescript
    # npm install -D typescript @types/node@18.18
    # https://github.com/DefinitelyTyped/DefinitelyTyped/issues/66934
    # npm install -D @types/mocha mocha
    # npm install -D @types/mocha@10.0.2
    # npm uninstall -D @types/jest
    # npm install -D @types/jest

    # docker run -it -v E:\Works\OpenSource\Demo\MachineLearning\ONNX\ONNXRuntime\WebAssembly:/project -w /project/onnxruntime/js/web --rm emscripten/emsdk:3.1.57 /bin/bash

    # docker run -it -v E:\Works\OpenSource\Demo\MachineLearning\ONNX\ONNXRuntime\WebAssembly:/project -w /project/onnxruntime/js/web --rm node:18.18.0 /bin/bash
    # npm install -g typescript
    # npm install && npm install @types/jest @types/fs-extra && npm install esbuild jszip npmlog @types/minimist@1.2.2
    # npm run pull:wasm
    # npm run prepare 
    # npm run build 
}
Pop-Location