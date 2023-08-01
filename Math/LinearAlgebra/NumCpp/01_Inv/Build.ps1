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

$target = "NumCpp"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

$rootDir = Split-Path $current -Parent
$numCppInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath share | `
                    Join-Path -ChildPath NumCpp | `
                    Join-Path -ChildPath cmake

if (!(Test-path($numCppInstallDir)))
{
    Write-Host "${numCppInstallDir} is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${numCppInstallDir}" `
          -D NUMCPP_NO_USE_BOOST="ON" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${numCppInstallDir}" `
          -D NUMCPP_NO_USE_BOOST="ON" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${numCppInstallDir}" `
          -D NUMCPP_NO_USE_BOOST="ON" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

# run
$installDir = Join-Path "${installDir}" bin
if ($global:IsWindows)
{
    $exe = Join-Path "${installDir}" "Demo.exe"
    & "${exe}"
}
elseif ($global:IsLinux)
{
    $exe = Join-Path "${installDir}" "Demo"
    & "${exe}"
}
elseif ($global:IsMacOS)
{
    $exe = Join-Path "${installDir}" "Demo"
    & "${exe}"
}

#Push-Location $installDir
& "${exe}"
#Pop-Location