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
    python tools/ci_build/build.py --config ${Configuration} `
                                   --cmake_generator "Visual Studio 17 2022" `
                                   --parallel `
                                   --build_dir ${buildDir} `
                                   --skip_tests `
                                   --use_full_protobuf `
                                   --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $deps = @()
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\base"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\container"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\debugging"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\hash"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\numeric"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\profiling"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\strings"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\synchronization"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\time"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build\absl\types"; }
    $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Target = "flatbuffers-build"; }
    $deps += New-Object PSObject -Property @{ Name = "onnx";            Target = "onnx-build"; }
    $deps += New-Object PSObject -Property @{ Name = "protobuf";        Target = "protobuf-build"; }
    $deps += New-Object PSObject -Property @{ Name = "pytorch_cpuinfo"; Target = "pytorch_cpuinfo-build"; }
    $deps += New-Object PSObject -Property @{ Name = "re2";             Target = "re2-build"; }
    $exts = @(
        "*.lib"
        "*.pdb"
    )

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    $depsDir = Join-Path $artifactDir _deps
    foreach ($dep in $deps)
    {
        $depSourceDir = Join-Path $depsDir $dep.Target | Join-Path -ChildPath $Configuration
        $depsDestDir = Join-Path $depsInstallDir $dep.Name
        New-Item -Type Directory $depsDestDir -Force | Out-Null

        foreach ($ext in $exts)
        {
            $src = Join-Path $depSourceDir $ext
            Copy-Item "${src}" "${depsDestDir}" -Force
        }
    }
}
elseif ($global:IsMacOS)
{
    python3 tools/ci_build/build.py --config ${Configuration} `
                                    --parallel `
                                    --build_dir ${buildDir} `
                                    --skip_tests `
                                    --use_full_protobuf `
                                    --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $deps = @()
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/base"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/container"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/debugging"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/hash"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/numeric"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/profiling"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/strings"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/synchronization"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/time"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/types"; }
    $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Target = "flatbuffers-build"; }
    $deps += New-Object PSObject -Property @{ Name = "google_nsync";    Target = "google_nsync-build"; }
    $deps += New-Object PSObject -Property @{ Name = "onnx";            Target = "onnx-build"; }
    $deps += New-Object PSObject -Property @{ Name = "protobuf";        Target = "protobuf-build"; }
    $deps += New-Object PSObject -Property @{ Name = "pytorch_cpuinfo"; Target = "pytorch_cpuinfo-build"; }
    $deps += New-Object PSObject -Property @{ Name = "re2";             Target = "re2-build"; }
    $exts = @(
        "*.a"
    )

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    $depsDir = Join-Path $artifactDir _deps
    foreach ($dep in $deps)
    {
        $depSourceDir = Join-Path $depsDir $dep.Target
        $depsDestDir = Join-Path $depsInstallDir $dep.Name
        New-Item -Type Directory $depsDestDir -Force | Out-Null

        foreach ($ext in $exts)
        {
            $src = Join-Path $depSourceDir $ext
            Copy-Item "${src}" "${depsDestDir}" -Force
        }
    }
}
elseif ($global:IsLinux)
{
    python3 tools/ci_build/build.py --config ${Configuration} `
                                    --parallel `
                                    --build_dir ${buildDir} `
                                    --skip_tests `
                                    --use_full_protobuf `
                                    --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    $artifactDir = Join-Path $buildDir $Configuration
    cmake --install $artifactDir --config ${Configuration}

    $deps = @()
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/base"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/container"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/debugging"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/hash"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/numeric"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/profiling"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/strings"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/synchronization"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/time"; }
    $deps += New-Object PSObject -Property @{ Name = "abseil";          Target = "abseil_cpp-build/absl/types"; }
    $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Target = "flatbuffers-build"; }
    $deps += New-Object PSObject -Property @{ Name = "google_nsync";    Target = "google_nsync-build"; }
    $deps += New-Object PSObject -Property @{ Name = "onnx";            Target = "onnx-build"; }
    $deps += New-Object PSObject -Property @{ Name = "protobuf";        Target = "protobuf-build"; }
    $deps += New-Object PSObject -Property @{ Name = "pytorch_cpuinfo"; Target = "pytorch_cpuinfo-build"; }
    $deps += New-Object PSObject -Property @{ Name = "clog";            Target = "pytorch_cpuinfo-build/deps/clog"; }
    $deps += New-Object PSObject -Property @{ Name = "re2";             Target = "re2-build"; }
    $exts = @(
        "*.a"
    )

    $depsInstallDir = Join-Path $installDir lib | Join-Path -ChildPath deps
    $depsDir = Join-Path $artifactDir _deps
    foreach ($dep in $deps)
    {
        $depSourceDir = Join-Path $depsDir $dep.Target
        $depsDestDir = Join-Path $depsInstallDir $dep.Name
        New-Item -Type Directory $depsDestDir -Force | Out-Null

        foreach ($ext in $exts)
        {
            $src = Join-Path $depSourceDir $ext
            Copy-Item "${src}" "${depsDestDir}" -Force
        }
    }
}
Pop-Location