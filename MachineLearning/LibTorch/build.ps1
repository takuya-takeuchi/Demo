#***************************************
#Arguments
#%1: Build Target (win/linux/osx/ios/android)
#%2: Architecture (x86_64/arm64)
#%3: Build Configuration (Release/Debug/RelWithDebInfo/MinSizeRel)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Target,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Architecture,

   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $Configuration
)

$buildTarget = "pytorch"

$os = $Target
$configuration = $Configuration
$architecture = $Architecture

$TargetArray =
@(
   "win",
   "linux",
   "osx",
   "ios",
   "android"
)

$ConfigurationArray =
@(
   "Debug",
   "Release",
   "RelWithDebInfo",
   "MinSizeRel"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64"
)

if ($TargetArray.Contains($os) -eq $False)
{
   $candidate = $TargetArray.Keys -join "/"
   Write-Host "Error: Specify Target [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray.Keys -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray.Keys -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot

switch ($os)
{
    "win"
    {
        # linked binary crashes... so use 1.8.2
        $version = "v1.8.2"
        $commit = "e0495a7"
    }
    "linux"
    {
        $version = "v1.12.0"
        $commit = "67ece03"
    }
    "osx"
    {
        $version = "v1.12.0"
        $commit = "67ece03"
    }
    "ios"
    {
        $version = "v1.12.0"
        $commit = "67ece03"
    }
    "android"
    {
    }
}

$sourceDir = Join-Path $current ${buildTarget}
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $architecture
$build_dirname = "build_${os}_${architecture}"
$env:TORCH_BUILD_DIRECTORY_NAME = $build_dirname

New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir

# restore
git checkout .
git submodule sync
git submodule update --init --recursive .

git fetch --all
git checkout ${commit}

$patch = Join-Path $current patch | `
         Join-Path -ChildPath $version | `
         Join-Path -ChildPath "*"
Copy-Item -Recurse $patch $sourceDir -Force

switch ($os)
{
    "win"
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
        $env:CMAKE_BUILD_TYPE=${configuration}
        $env:BUILD_CAFFE2_OPS="OFF"
        $env:BUILD_DOCS="OFF"
        $env:BUILD_PYTHON="OFF"
        $env:BUILD_SHARED_LIBS="OFF"
        $env:BUILD_TEST="OFF"
        $env:USE_DISTRIBUTED="OFF"
        $env:USE_CUDA="OFF"
        $env:USE_KINETO="OFF"
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
        cmake --build $build_dirname --target install --config ${configuration}

        # cmake file
        $destCmake = Join-Path $installDir share | `
                     Join-Path -ChildPath cmake | `
                     Join-Path -ChildPath Torch
        New-Item -Type Directory $destCmake -Force | Out-Null
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfig.cmake
        Copy-Item $buildCmake $destCmake -Force
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfigVersion.cmake
        Copy-Item $buildCmake $destCmake -Force

        # library
        $buildLib = Join-Path $sourceDir $build_dirname | `
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
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath aten | `
                        Join-Path -ChildPath src | `
                        Join-Path -ChildPath ATen
        Copy-Item $buildInclude $destInclude -Recurse -Force

        $buildInclude = Join-Path $sourceDir c10
        Copy-Item $buildInclude $destInclude -Recurse -Force
        $destInclude = Join-Path $installDir include | `
                        Join-Path -ChildPath c10
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath c10 | `
                        Join-Path -ChildPath macros
        Copy-Item $buildInclude $destInclude -Recurse -Force
    }
    "linux"
    {
        $env:CMAKE_BUILD_TYPE=${configuration}
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
        $env:USE_PYTORCH_QNNPACK="OFF"
        $env:USE_SYSTEM_SLEEF="OFF"
        $env:USE_NNPACK="OFF"
        $env:USE_TENSORRT="OFF"
        $env:USE_XNNPACK="OFF"
        $env:USE_SYSTEM_LIBS="OFF"
        $env:USE_SYSTEM_ONNX="OFF"
        $env:USE_NINJA="ON"
        $env:ONNX_ML="OFF"
        python setup.py build --cmake-only
        cmake --build $build_dirname --target install --config ${configuration}

        # cmake file
        $destCmake = Join-Path $installDir share | `
                     Join-Path -ChildPath cmake | `
                     Join-Path -ChildPath Torch
        New-Item -Type Directory $destCmake -Force | Out-Null
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfig.cmake
        Copy-Item $buildCmake $destCmake -Force
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfigVersion.cmake
        Copy-Item $buildCmake $destCmake -Force

        # library
        $buildLib = Join-Path $sourceDir $build_dirname | `
                    Join-Path -ChildPath lib
        Copy-Item $buildLib $installDir -Recurse -Force
        $buildLib = Join-Path $sourceDir $build_dirname | `
                    Join-Path -ChildPath sleef | `
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
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath aten | `
                        Join-Path -ChildPath src | `
                        Join-Path -ChildPath ATen
        Copy-Item $buildInclude $destInclude -Recurse -Force

        $buildInclude = Join-Path $sourceDir c10
        Copy-Item $buildInclude $destInclude -Recurse -Force
        $destInclude = Join-Path $installDir include | `
                       Join-Path -ChildPath c10
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath c10 | `
                        Join-Path -ChildPath macros
        Copy-Item $buildInclude $destInclude -Recurse -Force
    }
    "osx"
    {
        $env:CMAKE_BUILD_TYPE=${configuration}
        $env:BUILD_CAFFE2_OPS="OFF"
        $env:BUILD_DOCS="OFF"
        $env:BUILD_PYTHON="OFF"
        $env:BUILD_SHARED_LIBS="OFF"
        $env:BUILD_TEST="OFF"
        $env:USE_DISTRIBUTED="OFF"
        $env:USE_CUDA="OFF"
        $env:USE_OPENMP="OFF"
        $env:USE_KINETO="OFF"
        $env:USE_MKLDNN="OFF"
        $env:USE_FBGEMM="OFF"
        $env:USE_QNNPACK="OFF"
        $env:USE_EIGEN_FOR_BLAS="OFF"
        $env:USE_PYTORCH_QNNPACK="OFF"
        $env:USE_NNPACK="OFF"
        $env:USE_TENSORRT="OFF"
        $env:USE_XNNPACK="OFF"
        $env:USE_SYSTEM_LIBS="OFF"
        $env:USE_SYSTEM_ONNX="OFF"
        $env:USE_NINJA="ON"
        $env:USE_MPS="ON"
        $env:ONNX_ML="OFF"
        $env:MACOSX_DEPLOYMENT_TARGET=11.0
        $env:CMAKE_OSX_ARCHITECTURES="${architecture}"
        if ("${architecture}" -eq "arm64")
        {
            $env:USE_SLEEF_FOR_ARM_VEC256="OFF"
        }

        python setup.py build --cmake-only
        cmake --build $build_dirname --target install --config ${configuration}

        # cmake file
        $destCmake = Join-Path $installDir share | `
                     Join-Path -ChildPath cmake | `
                     Join-Path -ChildPath Torch
        New-Item -Type Directory $destCmake -Force | Out-Null
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfig.cmake
        Copy-Item $buildCmake $destCmake -Force
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfigVersion.cmake
        Copy-Item $buildCmake $destCmake -Force

        # library
        $buildLib = Join-Path $sourceDir $build_dirname | `
                    Join-Path -ChildPath lib
        Copy-Item $buildLib $installDir -Recurse -Force
        $buildLib = Join-Path $sourceDir $build_dirname | `
                    Join-Path -ChildPath sleef | `
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
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath aten | `
                        Join-Path -ChildPath src | `
                        Join-Path -ChildPath ATen
        Copy-Item $buildInclude $destInclude -Recurse -Force

        $buildInclude = Join-Path $sourceDir c10
        Copy-Item $buildInclude $destInclude -Recurse -Force
        $destInclude = Join-Path $installDir include | `
                       Join-Path -ChildPath c10
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath c10 | `
                        Join-Path -ChildPath macros
        Copy-Item $buildInclude $destInclude -Recurse -Force
    }
    "ios"
    {
        $env:CMAKE_BUILD_TYPE=${configuration}
        $env:BUILD_CAFFE2_OPS="OFF"
        $env:BUILD_DOCS="OFF"
        $env:BUILD_PYTHON="OFF"
        $env:BUILD_SHARED_LIBS="OFF"
        $env:BUILD_TEST="OFF"
        $env:USE_DISTRIBUTED="OFF"
        $env:USE_CUDA="OFF"
        $env:USE_OPENMP="OFF"
        $env:USE_KINETO="OFF"
        $env:USE_MKLDNN="OFF"
        $env:USE_FBGEMM="OFF"
        $env:USE_QNNPACK="OFF"
        $env:USE_EIGEN_FOR_BLAS="OFF"
        $env:USE_PYTORCH_QNNPACK="OFF"
        $env:USE_NNPACK="OFF"
        $env:USE_TENSORRT="OFF"
        $env:USE_XNNPACK="OFF"
        $env:USE_SYSTEM_LIBS="OFF"
        $env:USE_SYSTEM_ONNX="OFF"
        $env:USE_NINJA="ON"
        $env:USE_MPS="ON"
        $env:ONNX_ML="OFF"
        $env:MACOSX_DEPLOYMENT_TARGET=11.0
        $env:CMAKE_SYSTEM_NAME="iOS"
        $env:CMAKE_OSX_SYSROOT="iphoneos"
        if ("${architecture}" -eq "arm64")
        {
            $env:USE_SLEEF_FOR_ARM_VEC256="OFF"
        }

        python setup.py build --cmake-only
        cmake --build $build_dirname --target install --config ${configuration}

        # cmake file
        $destCmake = Join-Path $installDir share | `
                     Join-Path -ChildPath cmake | `
                     Join-Path -ChildPath Torch
        New-Item -Type Directory $destCmake -Force | Out-Null
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfig.cmake
        Copy-Item $buildCmake $destCmake -Force
        $buildCmake = Join-Path $sourceDir $build_dirname | `
                      Join-Path -ChildPath TorchConfigVersion.cmake
        Copy-Item $buildCmake $destCmake -Force

        # library
        $buildLib = Join-Path $sourceDir $build_dirname | `
                    Join-Path -ChildPath lib
        Copy-Item $buildLib $installDir -Recurse -Force
        $buildLib = Join-Path $sourceDir $build_dirname | `
                    Join-Path -ChildPath sleef | `
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
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath aten | `
                        Join-Path -ChildPath src | `
                        Join-Path -ChildPath ATen
        Copy-Item $buildInclude $destInclude -Recurse -Force

        $buildInclude = Join-Path $sourceDir c10
        Copy-Item $buildInclude $destInclude -Recurse -Force
        $destInclude = Join-Path $installDir include | `
                       Join-Path -ChildPath c10
        $buildInclude = Join-Path $sourceDir $build_dirname | `
                        Join-Path -ChildPath c10 | `
                        Join-Path -ChildPath macros
        Copy-Item $buildInclude $destInclude -Recurse -Force
    }
    "android"
    {
    }
}

# remove files except for *.h
$include = Join-Path $installDir include 
Get-ChildItem -Path $include -Recurse -File | Where-Object { $_.Extension -ne ".h" } | Remove-Item -Force -Recurse
# remove empty directories
do {
    $directories = Get-ChildItem -Path $include  -Recurse -Directory | Where-Object { !(Get-ChildItem -Path $_.FullName) }
    $deleted = $false
    foreach ($dir in $directories)
    {
        try
        {
            Remove-Item -Path $dir.FullName -Force
            $deleted = $true
        }
        catch
        {
        }
    }
} while ($deleted)

Pop-Location
