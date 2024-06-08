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

$target = "rapidjson"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

git submodule update --init --recursive .

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D RAPIDJSON_BUILD_DOC=OFF `
          -D RAPIDJSON_BUILD_EXAMPLES=OFF `
          -D RAPIDJSON_BUILD_TESTS=OFF `
          -D RAPIDJSON_BUILD_THIRDPARTY_GTEST=OFF `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D RAPIDJSON_BUILD_DOC=OFF `
          -D RAPIDJSON_BUILD_EXAMPLES=OFF `
          -D RAPIDJSON_BUILD_TESTS=OFF `
          -D RAPIDJSON_BUILD_THIRDPARTY_GTEST=OFF `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D RAPIDJSON_BUILD_DOC=OFF `
          -D RAPIDJSON_BUILD_EXAMPLES=OFF `
          -D RAPIDJSON_BUILD_TESTS=OFF `
          -D RAPIDJSON_BUILD_THIRDPARTY_GTEST=OFF `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location