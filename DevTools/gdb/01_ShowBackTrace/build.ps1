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

$ConfigurationArray =
@(
   "Debug",
   "Release"
)

if ($ConfigurationArray.Contains($Configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Specify build configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot

# get os name
if ($global:IsLinux)
{
    $os = "linux"
}
else
{
    Write-Host "Unsupported OS" -ForegroundColor Red
    exit -1
}

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $Architecture | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install| `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $Architecture | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
    )
}
elseif ($global:IsLinux)
{    
    $env:OPENSSL_ROOT_DIR = "${OPENSSL_INSTALL_DIR}"
    $cmakeArgs += @(
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
    )
}

$cmakeArgs += @(
    "-D CMAKE_POLICY_VERSION_MINIMUM=3.5"
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location