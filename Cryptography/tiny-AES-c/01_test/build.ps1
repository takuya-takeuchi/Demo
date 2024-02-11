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
$root = Split-Path $PSScriptRoot -Parent

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

$target = "tiny-AES-c"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$targetDir = Join-Path $root install | `
             Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D CMAKE_BUILD_TYPE=$Configuration `
      -D TINY_AES_C_INSTALL_DIR=$targetDir `
      $sourceDir
cmake --build . --config ${Configuration} --target install
Pop-Location

# run
if ($global:IsWindows)
{
    $programDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath bin
    $program = Join-Path $programDir Test.exe
}
elseif ($global:IsMacOS)
{
    $programDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath bin
    $program = Join-Path $programDir Test
}
elseif ($global:IsLinux)
{
    $programDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath bin
    $program = Join-Path $programDir Test
}

Push-Location ${programDir}
& ${program}
Pop-Location