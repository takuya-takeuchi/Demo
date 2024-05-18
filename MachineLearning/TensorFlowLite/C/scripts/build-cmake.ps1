Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Version
)

$current = Split-Path $PSScriptRoot -Parent

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

$target = "tensorflow"
$version = $Version
$lang = "c"
$buildSystem = "cmake"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $lang | `
            Join-Path -ChildPath $buildSystem | `
            Join-Path -ChildPath $version
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $lang | `
              Join-Path -ChildPath $buildSystem | `
              Join-Path -ChildPath $Configuration | `
              Join-Path -ChildPath $version
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath $buildSystem

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir
git checkout $version
Pop-Location

Push-Location $buildDir

if ($global:IsWindows)
{
    $TFLITE = Join-Path $sourceDir tensorflow | `
              Join-Path -ChildPath lite | `
              Join-Path -ChildPath $lang
    cmake -G "Visual Studio 17 2022" -A x64 `
         -D CMAKE_BUILD_TYPE="${Configuration}" `
         -D BUILD_SHARED_LIBS=ON `
         -D TFLITE_C_BUILD_SHARED_LIBS=ON `
         "${TFLITE}"
    cmake --build . --config $Configuration --target install

    switch ($Version)
    {
        "v2.5.3"
        {
            $deps = @()
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\base"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\container"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\debugging"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\flags"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\hash"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\numeric"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\status"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\strings"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\synchronization"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\time"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps"; Target = "abseil-cpp-build\absl\types"; }
            $deps += New-Object PSObject -Property @{ Name = "farmhash";        Parent = "_deps"; Target = "farmhash-build"; }
            $deps += New-Object PSObject -Property @{ Name = "fft2d";           Parent = "_deps"; Target = "fft2d-build"; }
            $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Parent = "_deps"; Target = "flatbuffers-build"; }
            $deps += New-Object PSObject -Property @{ Name = "ruy";             Parent = "_deps"; Target = "ruy-build"; }
            $deps += New-Object PSObject -Property @{ Name = "xnnpack";         Parent = "_deps"; Target = "xnnpack-build"; }
            $deps += New-Object PSObject -Property @{ Name = "clog";            Parent = "";      Target = "clog"; }
            $deps += New-Object PSObject -Property @{ Name = "cpuinfo";         Parent = "";      Target = "cpuinfo"; }
            $deps += New-Object PSObject -Property @{ Name = "pthreadpool";     Parent = "";      Target = "pthreadpool"; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "";      Target = ""; }
        }
        "v2.10.1"
        {
            $deps = @()
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\base"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\container"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\debugging"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\flags"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\hash"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\numeric"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\profiling"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\status"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\strings"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\synchronization"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\time"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\types"; }
            $deps += New-Object PSObject -Property @{ Name = "cpuinfo";         Parent = "_deps";           Target = "cpuinfo-build"; }
            $deps += New-Object PSObject -Property @{ Name = "farmhash";        Parent = "_deps";           Target = "farmhash-build"; }
            $deps += New-Object PSObject -Property @{ Name = "fft2d";           Parent = "_deps";           Target = "fft2d-build"; }
            $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Parent = "_deps";           Target = "flatbuffers-build"; }
            $deps += New-Object PSObject -Property @{ Name = "ruy";             Parent = "_deps";           Target = "ruy-build\ruy"; }
            $deps += New-Object PSObject -Property @{ Name = "xnnpack";         Parent = "_deps";           Target = "xnnpack-build"; }
            $deps += New-Object PSObject -Property @{ Name = "pthreadpool";     Parent = "";                Target = "pthreadpool"; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "tensorflow-lite"; Target = ""; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "";                Target = ""; }
        }
        "v2.11.1"
        {
            $deps = @()
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\base"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\container"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\debugging"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\flags"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\hash"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\numeric"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\profiling"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\status"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\strings"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\synchronization"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\time"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\types"; }
            $deps += New-Object PSObject -Property @{ Name = "cpuinfo";         Parent = "_deps";           Target = "cpuinfo-build"; }
            $deps += New-Object PSObject -Property @{ Name = "farmhash";        Parent = "_deps";           Target = "farmhash-build"; }
            $deps += New-Object PSObject -Property @{ Name = "fft2d";           Parent = "_deps";           Target = "fft2d-build"; }
            $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Parent = "_deps";           Target = "flatbuffers-build"; }
            $deps += New-Object PSObject -Property @{ Name = "ruy";             Parent = "_deps";           Target = "ruy-build\ruy"; }
            $deps += New-Object PSObject -Property @{ Name = "xnnpack";         Parent = "_deps";           Target = "xnnpack-build"; }
            $deps += New-Object PSObject -Property @{ Name = "pthreadpool";     Parent = "";                Target = "pthreadpool"; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "tensorflow-lite"; Target = ""; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "";                Target = ""; }
        }
        "v2.12.1"
        {
            $deps = @()
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\base"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\container"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\debugging"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\flags"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\hash"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\numeric"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\profiling"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\status"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\strings"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\synchronization"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\time"; }
            $deps += New-Object PSObject -Property @{ Name = "abseil";          Parent = "_deps";           Target = "abseil-cpp-build\absl\types"; }
            $deps += New-Object PSObject -Property @{ Name = "cpuinfo";         Parent = "_deps";           Target = "cpuinfo-build"; }
            $deps += New-Object PSObject -Property @{ Name = "farmhash";        Parent = "_deps";           Target = "farmhash-build"; }
            $deps += New-Object PSObject -Property @{ Name = "fft2d";           Parent = "_deps";           Target = "fft2d-build"; }
            $deps += New-Object PSObject -Property @{ Name = "flatbuffers";     Parent = "_deps";           Target = "flatbuffers-build"; }
            $deps += New-Object PSObject -Property @{ Name = "ruy";             Parent = "_deps";           Target = "ruy-build\ruy"; }
            $deps += New-Object PSObject -Property @{ Name = "xnnpack";         Parent = "_deps";           Target = "xnnpack-build"; }
            $deps += New-Object PSObject -Property @{ Name = "pthreadpool";     Parent = "";                Target = "pthreadpool"; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "tensorflow-lite"; Target = ""; }
            $deps += New-Object PSObject -Property @{ Name = "";                Parent = "";                Target = ""; }
        }
    }

    $exts = @(
        "*.lib"
        "*.pdb"
    )

    $artifactDir = $buildDir

    $depsInstallDir = Join-Path $installDir lib
    foreach ($dep in $deps)
    {
        if ($dep.Parent -ne "")
        {
            $depsDir = Join-Path $artifactDir $dep.Parent
        }
        else
        {
            $depsDir = $artifactDir
        }

        if ($dep.Target -ne "")
        {
            $depSourceDir = Join-Path $depsDir $dep.Target | Join-Path -ChildPath $Configuration
        }
        else
        {
            $depSourceDir = Join-Path $depsDir $Configuration
        }

        if ($dep.Name -ne "")
        {
            $depsDestDir = Join-Path $depsInstallDir $dep.Name
        }
        else
        {
            $depsDestDir = $depsInstallDir
        }
        
        New-Item -Type Directory $depsDestDir -Force | Out-Null

        foreach ($ext in $exts)
        {
            $src = Join-Path $depSourceDir $ext
            if (!(Test-Path($src)))
            {
                continue
            }

            Copy-Item "${src}" "${depsDestDir}" -Force
        }
    }

    $exts = @(
        "*.dll"
        "*.pdb"
    )
    $depsInstallDir = Join-Path $installDir bin
    foreach ($dep in $deps)
    {
        if ($dep.Parent -ne "")
        {
            $depsDir = Join-Path $artifactDir $dep.Parent
        }
        else
        {
            $depsDir = $artifactDir
        }

        if ($dep.Target -ne "")
        {
            $depSourceDir = Join-Path $depsDir $dep.Target | Join-Path -ChildPath $Configuration
        }
        else
        {
            $depSourceDir = Join-Path $depsDir $Configuration
        }

        $depsDestDir = $depsInstallDir
        
        New-Item -Type Directory $depsDestDir -Force | Out-Null

        foreach ($ext in $exts)
        {
            $src = Join-Path $depSourceDir $ext
            #Write-Host "src: ${src}, depsDestDir: ${depsDestDir}" -ForegroundColor Green
            Copy-Item "${src}" "${depsDestDir}" -Force
        }
    }

    switch ($Version)
    {
        "v2.5.3"
        {
            $includes = @()
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api.h";              Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_experimental.h"; Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_types.h";        Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "common.h";             Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
        }
        "v2.10.1"
        {
            $includes = @()
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api.h";              Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_experimental.h"; Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_types.h";        Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "common.h";             Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
        }
        "v2.11.1"
        {
            $includes = @()
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api.h";              Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_experimental.h"; Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_types.h";        Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "common.h";             Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\core\c";   Name = "c_api.h";              Target = "lite\core\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
        }
        "v2.12.1"
        {
            $includes = @()
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api.h";              Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_experimental.h"; Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_types.h";        Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "common.h";             Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\core\c";   Name = "c_api.h";              Target = "lite\core\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
        }
    }

    if ($includes)
    {
        $includeInstallDir = Join-Path $installDir include
        foreach ($include in $includes)
        {
            $includeDestDir = Join-Path $includeInstallDir tensorflow | `
                                Join-Path -ChildPath $include.Target
            New-Item -Type Directory $includeDestDir -Force | Out-Null
    
            $includeSourceDir = Join-Path $sourceDir tensorflow | `
                                Join-Path -ChildPath $include.Source
            $src = Join-Path $includeSourceDir $include.Name
            Copy-Item "${src}" "${includeDestDir}" -Force
        }
    }
}
elseif ($global:IsMacOS)
{
}
elseif ($global:IsLinux)
{
}
Pop-Location