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

$target = "libtorch"

# # 1.7.0 can not build due to https://github.com/pytorch/pytorch/issues/48517
# $version = "v1.7.0"
# $commit = "e85d494"

# 1.8.2 
$version = "v1.8.2"
$commit = "e0495a7"

# $version = "v1.9.1"
# $commit = "dfbd030"

# $version = "v1.11.0"
# $commit = "bc2c6ed"

$sourceDir = Join-Path $current pytorch
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir

# restore
git submodule sync
git submodule update --init --recursive .

git fetch --all
git checkout ${commit}

$patch = Join-Path $current patch | `
         Join-Path -ChildPath $version | `
         Join-Path -ChildPath "*"
Copy-Item -Recurse $patch $sourceDir -Force

if ($global:IsWindows)
{
    function Call($batfile)
    {
        cmd.exe /c "call `"${batfile}`" && set > %temp%\vars.txt"
        Get-Content "${env:temp}\vars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }

    Call("C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat")

    $env:MSVC_Z7_OVERRIDE="OFF"
    $env:CMAKE_BUILD_TYPE=${Configuration}
    $env:BUILD_CAFFE2_OPS="OFF"
    $env:BUILD_DOCS="OFF"
    $env:BUILD_PYTHON="OFF"
    $env:BUILD_SHARED_LIBS="OFF"
    $env:BUILD_TEST="OFF"
    $env:USE_DISTRIBUTED="OFF"
    $env:USE_CUDA="OFF"
    $env:USE_OPENMP="OFF"
    $env:USE_MKLDNN="OFF"
    $env:USE_FBGEMM="OFF"
    $env:USE_QNNPACK="OFF"
    $env:USE_NNPACK="OFF"
    $env:USE_TENSORRT="OFF"
    $env:USE_XNNPACK="OFF"
    $env:USE_SYSTEM_LIBS="OFF"
    $env:USE_SYSTEM_ONNX="OFF"
    $env:USE_NINJA="ON"
    $env:ONNX_ML="OFF"
    $env:CAFFE2_USE_MSVC_STATIC_RUNTIME="ON"
    python setup.py build --cmake-only
    cmake --build build --target install --config ${Configuration}

    # cmake file
    $destCmake = Join-Path $installDir share | `
                 Join-Path -ChildPath cmake | `
                 Join-Path -ChildPath Torch
    New-Item -Type Directory $destCmake -Force | Out-Null
    $buildCmake = Join-Path $sourceDir build | `
                  Join-Path -ChildPath TorchConfig.cmake
    Copy-Item $buildCmake $destCmake -Force
    $buildCmake = Join-Path $sourceDir build | `
                  Join-Path -ChildPath TorchConfigVersion.cmake
    Copy-Item $buildCmake $destCmake -Force

    # library
    $buildLib = Join-Path $sourceDir build | `
                Join-Path -ChildPath lib
    Copy-Item $buildLib $installDir -Recurse -Force

    # header
    $destInclude = Join-Path $installDir include
    New-Item -Type Directory $destInclude -Force | Out-Null
    $buildInclude = Join-Path $sourceDir torch
    Copy-Item $buildInclude $destInclude -Recurse -Force

    $destInclude = Join-Path $installDir include
    $buildInclude = Join-Path $sourceDir aten | `
                    Join-Path -ChildPath src | `
                    Join-Path -ChildPath ATen
    Copy-Item $buildInclude $destInclude -Recurse -Force
    $buildInclude = Join-Path $sourceDir build | `
                    Join-Path -ChildPath aten | `
                    Join-Path -ChildPath src | `
                    Join-Path -ChildPath ATen
    Copy-Item $buildInclude $destInclude -Recurse -Force

    $buildInclude = Join-Path $sourceDir c10
    Copy-Item $buildInclude $destInclude -Recurse -Force
    $destInclude = Join-Path $installDir include | `
                   Join-Path -ChildPath c10
    $buildInclude = Join-Path $sourceDir build | `
                    Join-Path -ChildPath c10 | `
                    Join-Path -ChildPath macros
    Copy-Item $buildInclude $destInclude -Recurse -Force
}
elseif ($global:IsMacOS)
{
    $env:CMAKE_BUILD_TYPE=${Configuration}
    $env:BUILD_CAFFE2_OPS="OFF"
    $env:BUILD_DOCS="OFF"
    $env:BUILD_PYTHON="OFF"
    $env:BUILD_SHARED_LIBS="OFF"
    $env:BUILD_TEST="OFF"
    $env:USE_DISTRIBUTED="OFF"
    $env:USE_CUDA="OFF"
    $env:USE_OPENMP="OFF"
    $env:USE_MKLDNN="OFF"
    $env:USE_FBGEMM="OFF"
    $env:USE_QNNPACK="OFF"
    $env:USE_NNPACK="OFF"
    $env:USE_TENSORRT="OFF"
    $env:USE_XNNPACK="OFF"
    $env:USE_SYSTEM_LIBS="OFF"
    $env:USE_SYSTEM_ONNX="OFF"
    $env:USE_NINJA="ON"
    $env:ONNX_ML="OFF"
    python setup.py build --cmake-only
    cmake --build build --target install --config ${Configuration}
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CPPKAFKA_ENABLE_TESTS=OFF `
          $sourceDir
}

# remove files except for *.h
$include = Join-Path $installDir include 
Get-ChildItem -Path $include -Recurse -File | Where-Object { $_.Extension -ne ".h" } | Remove-Item -Force
# remove empty directories
Get-ChildItem -Path $targetDirectory -Recurse -Directory | Where-Object { (Get-ChildItem -Path $_.FullName) -eq $null } | Remove-Item -Force

Pop-Location