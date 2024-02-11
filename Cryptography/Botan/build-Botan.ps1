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

$target = "Botan"
$url = "https://github.com/randombit/botan"
$branch = "55952b2a8b9ec6ae4220404837bc6942ea03be04"

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

$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

if (!(Test-Path($target)))
{
    git clone $url
}

Push-Location $target
git checkout $branch

python configure.py --with-cmake-config

Push-Location $buildDir

if ($global:IsWindows)
{
    Call("C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat")
    nmake
    nmake install
}
elseif ($global:IsMacOS)
{
    make
    make install
}
elseif ($global:IsLinux)
{
    make
    make install
}

# cmake --build "${buildDir}" --config $Configuration

# Copy-Item $library $installDir -Force | Out-Null
# Copy-Item $headerH $installDir -Force | Out-Null
# Copy-Item $headerHpp $installDir -Force | Out-Null

Pop-Location
Pop-Location