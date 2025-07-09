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
$rootDir = Split-Path $current -Parent

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

$target = "PcapPlusPlus"
$shared = "static"
$sharedFlag = "OFF"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $shared
$installBinaryDir = Join-Path $installDir bin

if ($global:IsWindows)
{
    $pcapPlusPlusInstallDir = Join-Path $rootDir install | `
                              Join-Path -ChildPath $os | `
                              Join-Path -ChildPath PcapPlusPlus | `
                              Join-Path -ChildPath $shared | `
                              Join-Path -ChildPath lib | `
                              Join-Path -ChildPath cmake
    $npcapSdkInstallDir = Join-Path $rootDir "npcap-sdk"
}
else
{
    $pcapPlusPlusInstallDir = Join-Path $rootDir install | `
                              Join-Path -ChildPath $os | `
                              Join-Path -ChildPath PcapPlusPlus | `
                              Join-Path -ChildPath $shared | `
                              Join-Path -ChildPath lib | `
                              Join-Path -ChildPath cmake
}

if (!(Test-Path($pcapPlusPlusInstallDir)))
{
    Write-Host "'${pcapPlusPlusInstallDir}' is missing" -ForegroundColor Red
    exit
}

if (!(Test-Path($pcapPlusPlusInstallDir)))
{
    Write-Host "'${pcapPlusPlusInstallDir}' is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    # If you want to use self build openssl, you must override $env:OPENSSL_ROOT_DIR
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${pcapPlusPlusInstallDir}" `
          -D PCAP_ROOT="${npcapSdkInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${pcapPlusPlusInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${pcapPlusPlusInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location