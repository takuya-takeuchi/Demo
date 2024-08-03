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
   $Configuration
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

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$buildLog = Join-Path $buildDir build.log
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $Configuration
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake

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
    $env:EM_CONFIG=""
    chcp 65001
    python tools/ci_build/build.py --config ${Configuration} `
                                   --cmake_generator Ninja `
                                   --build_wasm_static_lib `
                                   --emsdk_version 3.1.44 `
                                   --enable_wasm_simd `
                                   --enable_wasm_threads `
                                   --use_webnn `
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
}
Pop-Location