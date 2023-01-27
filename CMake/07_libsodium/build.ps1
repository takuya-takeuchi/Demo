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

$target = "libsodium"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

if ($global:IsWindows)
{
    $libsodium_INCLUDE_DIRS = Join-Path $installDir $target | `
                              Join-Path -ChildPath include
    $libsodium_LIBRARY = Join-Path $installDir $target | `
                         Join-Path -ChildPath lib | `
                         Join-Path -ChildPath static | `
                         Join-Path -ChildPath libsodium.lib
}
else
{
    $libsodium_INCLUDE_DIRS = Join-Path $installDir $target | `
                              Join-Path -ChildPath include
    $libsodium_LIBRARY = Join-Path $installDir $target | `
                         Join-Path -ChildPath lib | `
                         Join-Path -ChildPath libsodium.a
}

Push-Location $buildDir
cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D CMAKE_PREFIX_PATH=${targetDir} `
      -D libsodium_INCLUDE_DIRS=${libsodium_INCLUDE_DIRS} `
      -D libsodium_LIBRARY=${libsodium_LIBRARY} `
      $sourceDir
cmake --build . --config ${Configuration}
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
else
{
    $programDir = Join-Path $current build | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program
    $program = Join-Path $programDir Test
}

Push-Location ${programDir}
& ${program}
Pop-Location