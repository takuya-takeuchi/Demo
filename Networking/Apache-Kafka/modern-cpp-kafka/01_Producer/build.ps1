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
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

$rootDir = Split-Path $current -Parent
$modernCppKafkaInstallDir = Join-Path $rootDir install | `
                            Join-Path -ChildPath $os | `
                            Join-Path -ChildPath $target

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
          -D CMAKE_TOOLCHAIN_FILE="${env:VCPKG_ROOT_DIR}\scripts\buildsystems\vcpkg.cmake" `
          -D VCPKG_TARGET_TRIPLET="${library_type}" `
          -D MODERN_CPP_KAFKA_INCLUDE_DIR="${modernCppKafkaInstallDir}/include" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    $cpp_kafka_include_dir = Join-Path ${cppkafkaInstallDir} include
    $cpp_kafka_libraries = Join-Path ${cppkafkaInstallDir} lib | `
                           Join-Path -ChildPath libcppkafka.a
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetDir}" `
          -D CPPKAFKA_INCLUDE_DIR="${cpp_kafka_include_dir}" `
          -D CPPKAFKA_LIBRARIES="${cpp_kafka_libraries}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $cpp_kafka_include_dir = Join-Path ${cppkafkaInstallDir} include
    $cpp_kafka_libraries = Join-Path ${cppkafkaInstallDir} lib | `
                           Join-Path -ChildPath libcppkafka.a
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetDir}" `
          -D CPPKAFKA_INCLUDE_DIR="${cpp_kafka_include_dir}" `
          -D CPPKAFKA_LIBRARIES="${cpp_kafka_libraries}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

# run
if ($global:IsWindows)
{
    $programDir = Join-Path $installDir bin
    $library_type = "x64-windows"
    Copy-Item "${env:VCPKG_ROOT_DIR}\installed\${library_type}\bin\rdkafka.dll" "${programDir}" -Force
    Copy-Item "${env:VCPKG_ROOT_DIR}\installed\${library_type}\bin\lz4.dll" "${programDir}" -Force
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