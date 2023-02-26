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

$target = "modern-cpp-kafka"

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

    $library_type = "x64-windows"
    $vcpkg_base_directory = "${env:VCPKG_ROOT_DIR}\installed\${library_type}"
    $Env:GTEST_ROOT="${vcpkg_base_directory}"
    $Env:BOOST_ROOT="${vcpkg_base_directory}"
    $Env:LIBRDKAFKA_INCLUDE_DIR="${vcpkg_base_directory}\include\"
    $Env:LIBRDKAFKA_LIBRARY_DIR="${vcpkg_base_directory}\lib\"
    $Env:RAPIDJSON_INCLUDE_DIRS="${vcpkg_base_directory}\include\"

    # copy
    $boost_program_option = Get-ChildItem "${vcpkg_base_directory}\lib\boost_program_options-vc*-mt.lib"
    Copy-Item "${boost_program_option}" "${vcpkg_base_directory}\lib\boost_program_options.lib"

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_TOOLCHAIN_FILE="${toolchain}" `
          -D CPPKAFKA_ENABLE_TESTS=OFF `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CPPKAFKA_ENABLE_TESTS=OFF `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CPPKAFKA_ENABLE_TESTS=OFF `
          -D BOOST_ROOT="/usr/lib/x86_64-linux-gnu/cmake/Boost-1.74.0" `
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