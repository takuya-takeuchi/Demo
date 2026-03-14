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
$copnfigPath = Join-Path $current "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json

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
$version = $config.onnxruntime.version
$cudaVersion = "11.8"
$cudaVersionEnv = "11_8"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target-cuda | `
            Join-Path -ChildPath $version
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target-cuda | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $target

git submodule update --init --recursive .

if ($global:IsWindows)
{
    $env:CUDA_PATH = [System.Environment]::GetEnvironmentVariable("CUDA_PATH_V${cudaVersionEnv}", 'Machine')
    $cudaHome = $env:CUDA_PATH
    if (!(Test-Path($cudaHome)))
    {
        Write-Host "[Error] ${cudaHome}' is missing" -ForegroundColor Red
        exit
    }
    
    python tools/ci_build/build.py --config ${Configuration} `
                                   --cmake_generator "Visual Studio 17 2022" `
                                   --parallel `
                                   --build_dir ${buildDir} `
                                   --skip_tests `
                                   --skip_onnx_tests `
                                   --use_full_protobuf `
                                   --use_cuda `
                                   --cudnn_home "${cudaHome}" `
                                   --cuda_home "${cudaHome}" `
                                   --cuda_version $cudaVersion `
                                   --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir onnxruntime_BUILD_UNIT_TESTS=OFF

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    New-Item -Type Directory $depsInstallDir -Force | Out-Null
    $depsDir = Join-Path $artifactDir _deps
    Get-ChildItem -Path $depsDir -Recurse -Filter "*.lib" | Copy-Item -Destination $depsInstallDir
}
elseif ($global:IsLinux)
{
    python3 tools/ci_build/build.py --config ${Configuration} `
                                    --parallel `
                                    --build_dir ${buildDir} `
                                    --skip_tests `
                                    --skip_onnx_tests `
                                    --use_full_protobuf `
                                    --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir onnxruntime_BUILD_UNIT_TESTS=OFF

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    New-Item -Type Directory $depsInstallDir -Force | Out-Null
    $depsDir = Join-Path $artifactDir _deps
    Get-ChildItem -Path $depsDir -Recurse -Filter "*.a" | Copy-Item -Destination $depsInstallDir
}
Pop-Location