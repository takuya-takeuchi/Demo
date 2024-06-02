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

$target = "harfbuzz"
$shared = "static"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared

$freetype_installDir = Join-Path $current install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath freetype | `
                       Join-Path -ChildPath $shared
$freetype_include_dir = Join-Path $freetype_installDir include
$freetype_lib_dir = Join-Path $freetype_installDir lib

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $patch = Join-Path $current patch | `
             Join-Path -ChildPath $target | `
             Join-Path -ChildPath $os | `
             Join-Path -ChildPath "*"
    Copy-Item -Recurse $patch $sourceDir -Force

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PROJECT_INCLUDE=custom_options.cmake `
          -D BUILD_SHARED_LIBS=OFF `
          -D CMAKE_PREFIX_PATH="${freetype_installDir}" `
          -D HB_HAVE_FREETYPE=ON `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D CMAKE_PREFIX_PATH="${freetype_installDir}" `
          -D HB_HAVE_FREETYPE=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D CMAKE_PREFIX_PATH="${freetype_installDir}" `
          -D HB_HAVE_FREETYPE=ON `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location