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

$target = "curl"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH=${targetDir} `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH=${targetDir} `
          -D OPENSSL_ROOT_DIR=/usr/local/opt/openssl `
          -D OPENSSL_LIBRARIES=/usr/local/opt/openssl/lib `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH=${targetDir} `
          $sourceDir
}
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

Push-Location ${programDir}
& ${program} "https://phet-dev.colorado.edu/html/build-an-atom/0.0.0-3/simple-text-only-test-page.html"
Pop-Location