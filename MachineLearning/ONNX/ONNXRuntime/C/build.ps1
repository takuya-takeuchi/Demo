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

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $version
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $target

git fetch -ap
git checkout $version
git submodule update --init --recursive .

if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vs = "C:\Program Files\Microsoft Visual Studio\2022"
        $path = "${vs}\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Professional\VC\Auxiliary\Build\vcvars64.bat"
        }
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Community\VC\Auxiliary\Build\vcvars64.bat"
        }

        Write-Host "Use: ${path}" -ForegroundColor Green

        cmd.exe /c "call `"${path}`" && set > %temp%\vcvars.txt"
        Get-Content "${env:temp}\vcvars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    CallVisualStudioDeveloperConsole

    $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded"
    if ($Configuration -eq "Debug")
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY += "Debug"
    }

    # Do not speficy onnxruntime_BUILD_UNIT_TESTS. It occurs re2 is missing
    # Refer to https://github.com/microsoft/onnxruntime/issues/22513
    python tools/ci_build/build.py --config ${Configuration} `
                                   --cmake_generator "Visual Studio 17 2022" `
                                   --enable_msvc_static_runtime `
                                   --parallel `
                                   --build_dir ${buildDir} `
                                   --skip_tests `
                                   --skip_onnx_tests `
                                   --use_full_protobuf `
                                   --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir `
                                                         CMAKE_MSVC_RUNTIME_LIBRARY="${CMAKE_MSVC_RUNTIME_LIBRARY}"

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    New-Item -Type Directory $depsInstallDir -Force | Out-Null
    $depsDir = Join-Path $artifactDir _deps
    Get-ChildItem -Path $depsDir -Recurse -Filter "*.lib" | Copy-Item -Destination $depsInstallDir
}
elseif ($global:IsMacOS)
{
    # Do not speficy onnxruntime_BUILD_UNIT_TESTS. It occurs re2 is missing
    # Refer to https://github.com/microsoft/onnxruntime/issues/22513
    python3 tools/ci_build/build.py --config ${Configuration} `
                                    --parallel `
                                    --build_dir ${buildDir} `
                                    --skip_tests `
                                    --skip_onnx_tests `
                                    --use_full_protobuf `
                                    --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    New-Item -Type Directory $depsInstallDir -Force | Out-Null
    $depsDir = Join-Path $artifactDir _deps
    Get-ChildItem -Path $depsDir -Recurse -Filter "*.a" | Copy-Item -Destination $depsInstallDir
}
elseif ($global:IsLinux)
{
    python3 tools/ci_build/build.py --config ${Configuration} `
                                    --parallel `
                                    --build_dir ${buildDir} `
                                    --skip_tests `
                                    --skip_onnx_tests `
                                    --use_full_protobuf `
                                    --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir `
                                                          onnxruntime_BUILD_UNIT_TESTS=OFF

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    New-Item -Type Directory $depsInstallDir -Force | Out-Null
    $depsDir = Join-Path $artifactDir _deps
    Get-ChildItem -Path $depsDir -Recurse -Filter "*.a" | Copy-Item -Destination $depsInstallDir
}
Pop-Location