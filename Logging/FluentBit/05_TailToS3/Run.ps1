#***************************************
#Arguments
#   bucketName: Amazon S3 bucket name
#       region: Amazon S3 region
#***************************************
Param([Parameter(
      Mandatory=$True,
      Position = 1
      )][string]
      $bucketName,

      [Parameter(
      Mandatory=$True,
      Position = 2
      )][string]
      $region
)

$current = $PSScriptRoot
if ($global:IsWindows)
{
    $os = "windows"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}
else
{
    Write-Host "Error: This plaform is not support" -ForegroundColor Red
    exit -1
}
$root = Split-Path $current -Parent
$base = Join-Path $root "00_GetStarted" | Join-Path -ChildPath $os

$configPath = Join-Path $base "config.json"
$config = Get-Content -Path "${configPath}" | ConvertFrom-Json

$target = $config.target
$version = $config.version
$package = $config.serviceName

$config = Join-Path $current "fluent-bit-${os}.conf"

Write-Host "Starting ${target}..." -ForegroundColor Blue
if ($global:IsWindows)
{
    $packageDir = Join-Path $base $target | `
                  Join-Path -ChildPath $version
    if (!(Test-Path("${packageDir}")))
    {
        $scriptPath = Join-Path $base Download.ps1
        Write-Host "Please invoke '${scriptPath}' before run this script" -ForegroundColor Red
        exit
    }
    
    $binary = Join-Path $packageDir bin | `
              Join-Path -ChildPath "fluent-bit.exe"
    $config = Join-Path $current "fluent-bit-${os}.conf"

    # Set environmental variables to recognize current directory by fluent-bit
    # PROJECT_ROOT is used in fluent-bit-windows.conf
    $env:PROJECT_ROOT = $current
    $env:S3_BUCKET_NAME = $bucketName
    $env:S3_REGION = $region
    & "${binary}" -c "${config}"
}
elseif ($global:IsMacOS)
{
    # Set environmental variables to recognize current directory by fluent-bit
    # PROJECT_ROOT is used in fluent-bit-osx.conf
    $env:PROJECT_ROOT = $current
    $env:S3_BUCKET_NAME = $bucketName
    $env:S3_REGION = $region
    & "${package}"-c "${config}"
}
elseif ($global:IsLinux)
{
    # Set environmental variables to recognize current directory by fluent-bit
    # PROJECT_ROOT is used in fluent-bit-linux.conf
    $env:PROJECT_ROOT = $current
    $env:S3_BUCKET_NAME = $bucketName
    $env:S3_REGION = $region
    & "/opt/fluent-bit/bin/fluent-bit"-c "${config}"
}