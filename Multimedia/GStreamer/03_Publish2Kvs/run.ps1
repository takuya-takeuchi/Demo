#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#%2: Stream Name
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
   $StreamName
)

function Join-PathArray {
    [CmdletBinding()]
    param (
        [Parameter(Mandatory = $true, ValueFromPipeline = $true)]
        [string[]]$PathElements
    )

    process {
        if ($PathElements.Count -eq 0) { return }
        $result = $PathElements[0]
        for ($i = 1; $i -lt $PathElements.Count; $i++) { $result = Join-Path -Path $result -ChildPath $PathElements[$i] }
        return $result
    }
}

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$configPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json

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

$target = "gstreamer-kvs"
$version = $config.gstreamer.version

# build
$sourceDir = $current
$buildDir = Join-PathArray -PathElements @($current, "build", $os, $target, "program", $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os)
$installBinaryDir = Join-PathArray -PathElements @($installDir, "bin")
$targetInstallDir = Join-PathArray -PathElements @($rootDir, "install", $os, $target, $version, $Configuration)
if (!(Test-Path(${targetInstallDir})))
{
    Write-Host "[Error] ${targetInstallDir} is missing" -ForegroundColor Red
    return
}

$target = "amazon-kinesis-video-streams-producer-sdk-cpp"
$kvsVersion = $config."${target}".version
$kvsTarget = "kvs-sdk-cpp"

if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $env:GST_BASE = $targetInstallDir
    $lib = Join-PathArray -PathElements @($env:GST_BASE, "lib")
    $bin = Join-PathArray -PathElements @($env:GST_BASE, "bin")
    
    $env:GSTREAMER_VERSION = $version
    $env:GST_PLUGIN_SCANNER = Join-PathArray -PathElements @($env:GST_BASE, "libexec", "gstreamer-1.0", "gst-plugin-scanner")
    $pluginDir = Join-PathArray -PathElements @($lib, "gstreamer-1.0")
    $depenenciesDir = Join-PathArray -PathElements @($rootDir, $kvsTarget, "open-source", "local", "lib")
    $env:GST_PLUGIN_PATH = "${pluginDir}:"
    $env:DYLD_LIBRARY_PATH = "${lib}:${depenenciesDir}:${rootDir}/build/osx/kvs-sdk-cpp/v3.6.0/Release:${rootDir}/build/osx/kvs-sdk-cpp/v3.6.0/Release/dependency/libkvscproducer/kvscproducer-src:${env:DYLD_LIBRARY_PATH}"
    $env:PATH = "${bin}:${env:PATH}"

    $demo = Join-PathArray -PathElements @($installBinaryDir, "Demo")
}
elseif ($global:IsLinux)
{
    $env:GST_BASE = $targetInstallDir
    $lib = Join-PathArray -PathElements @($env:GST_BASE, "lib", "x86_64-linux-gnu")
    $bin = Join-PathArray -PathElements @($env:GST_BASE, "bin")
    $pluginDir = Join-PathArray -PathElements @($lib, "gstreamer-1.0")
    $kvsPluginDir = Join-PathArray -PathElements @($rootDir, "install", $os, $kvsTarget, $kvsVersion, $Configuration, "lib")

    $env:GSTREAMER_VERSION = $version
    $env:GST_PLUGIN_SCANNER = Join-PathArray -PathElements @($env:GST_BASE, "libexec", "gstreamer-1.0", "gst-plugin-scanner")
    $env:GST_PLUGIN_PATH = "${pluginDir}:${kvsPluginDir}"
    $env:LD_LIBRARY_PATH = "${lib}:${env:LD_LIBRARY_PATH}"
    $env:PATH = "${env:GST_BASE}/bin:${env:PATH}"

    $demo = Join-PathArray -PathElements @($installBinaryDir, "Demo")
}

& "${demo}" "sample-5s.mp4" $StreamName
