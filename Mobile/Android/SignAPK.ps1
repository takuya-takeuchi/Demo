Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $apkPath
)

$current = $PSScriptRoot

if (!(Test-Path(${apkPath})))
{
    Write-Host "apkPath: ${apkPath} is missing" -ForegroundColor Red
    exit
}

if ([String]::IsNullOrEmpty(${env:ANDROID_HOME}) -Or !(Test-Path(${env:ANDROID_HOME})))
{
    Write-Host "ANDROID_HOME: ${env:ANDROID_HOME} is missing" -ForegroundColor Red
    exit
}

Write-Host "ANDROID_HOME: '${env:ANDROID_HOME}'" -ForegroundColor Green

$buildtools = Join-Path $env:ANDROID_HOME build-tools

# find apksigner
if ($global:IsWindows)
{
    $apksignerName = "apksigner.bat"
}
else
{
    $apksignerName = "apksigner"
}

$files = Get-ChildItem "${buildtools}" -include "${apksignerName}" -Recurse | Sort-Object
$apksigner = $files[$files.Length - 1]
Write-Host "apksigner: '${apksigner}'" -ForegroundColor Green

$apkNewPath = $apkPath.Replace("app-debug", "app-debug-signed")
$apkNewPath = $apkNewPath.Replace("app-release-unsigned", "app-release-signed")

$key = Join-Path $current debug.keystore
& "${apksigner}" sign -ks "${key}" --out "${apkNewPath}" "${apkPath}"