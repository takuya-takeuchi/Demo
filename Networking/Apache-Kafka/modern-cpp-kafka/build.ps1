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

$target = "cppkafka"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    if (!($env:VCPKG_ROOT_DIR))
    {
        Write-Host "VCPKG_ROOT_DIR environmental variable is missing" -ForegroundColor Red
        return
    }

    $toolchain = "${env:VCPKG_ROOT_DIR}\scripts\buildsystems\vcpkg.cmake"
    if (!(Test-Path(${toolchain})))
    {
        Write-Host "${toolchain} is missing" -ForegroundColor Red
        return
    }

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetDir}" `
          -D CMAKE_TOOLCHAIN_FILE="${env:VCPKG_ROOT_DIR}\scripts\buildsystems\vcpkg.cmake" `
          -D VCPKG_TARGET_TRIPLET="x64-windows-static" `
          -D CPPKAFKA_BUILD_SHARED=OFF `
          -D CPPKAFKA_DISABLE_TESTS=ON `
          -D CPPKAFKA_DISABLE_EXAMPLES=ON `
          -D CPPKAFKA_RDKAFKA_STATIC_LIB=ON `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetDir}" `
          -D CPPKAFKA_BUILD_SHARED=OFF `
          -D CPPKAFKA_DISABLE_TESTS=ON `
          -D CPPKAFKA_DISABLE_EXAMPLES=ON `
          -D CPPKAFKA_RDKAFKA_STATIC_LIB=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetDir}" `
          -D CPPKAFKA_BUILD_SHARED=OFF `
          -D CPPKAFKA_DISABLE_TESTS=ON `
          -D CPPKAFKA_DISABLE_EXAMPLES=ON `
          -D CPPKAFKA_RDKAFKA_STATIC_LIB=ON `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

# run
if ($global:IsWindows)
{
    $programDir = Join-Path $current build | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program | `
                  Join-Path -ChildPath ${Configuration}
    $program = Join-Path $programDir Test.exe
}
elseif ($global:IsMacOS)
{
    $programDir = Join-Path $current build | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program
    $program = Join-Path $programDir Test
}
elseif ($global:IsLinux)
{
    $programDir = Join-Path $current build | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program
    $program = Join-Path $programDir Test
}