#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#%2: OpenSSL version (e.g. 1.1.1w, 3.4.0)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration,
   
   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $OpenSSLVersion
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
$openSSLInstallDir = Join-Path $rootDir install | `
                     Join-Path -ChildPath $os | `
                     Join-Path -ChildPath openssl | `
                     Join-Path -ChildPath ${OpenSSLVersion} | `
                     Join-Path -ChildPath $shared

if (!(Test-Path($openSSLInstallDir)))
{
    Write-Host "'${openSSLInstallDir}' is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $env:OPENSSL_ROOT_DIR = "${openSSLInstallDir}"
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openSSLInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openSSLInstallDir}" `
          -D CMAKE_OSX_ARCHITECTURES="arm64" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openSSLInstallDir}" `
          -D OPENSSL_CRYPTO_LIBRARY="${openSSLInstallDir}/lib64/libcrypto.a" `
          -D OPENSSL_SSL_LIBRARY="${openSSLInstallDir}/lib64/libcrypto.a" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location