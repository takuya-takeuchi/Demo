#***************************************
#Arguments
#%1: Gpl Type (switch)
#***************************************
Param
(
    [Parameter()]
    [Switch]
    $Gpl
)

$current = $PSScriptRoot

$configPath = Join-Path $current "build-config.json"
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
else
{
    Write-Host "This platform is not supported" -ForegroundColor Red
    exit
}

$target = "FFmpeg"
$version = $config.ffmpeg.version
$majorVersion = $config.ffmpeg.majorVersion
$baseUrl = "https://github.com/BtbN/FFmpeg-Builds/releases/download"
$tag = $config.ffmpeg.tag

$gplType = $Gpl ? "gpl" : "lgpl"
Write-Host "Download ${gplType} version" -ForegroundColor Yello

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $baseName = "ffmpeg-${version}-win64-${gplType}-${majorVersion}"
    $sha1 = $config.ffmpeg.sha256.${gplType}.win
    $file = "${baseName}.zip"
}
elseif ($global:IsMacOS)
{
    Write-Host "This platform is not supported" -ForegroundColor Red
    exit
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $baseName = "ffmpeg-${version}-linux64-${gplType}-${majorVersion}"
    $sha1 = $config.ffmpeg.sha256.${gplType}.linux
    $file = "${baseName}.tar.xz"
}

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $gplType

$path = ""
$url = "${baseUrl}/${tag}/${file}"
$path = Join-Path $current $file
$exist = Test-Path(${path})
if ($exist)
{
    $hash = (Get-FileHash ${file} -Algorithm SHA256).hash
    $exist = $sha1 -eq $hash
    if ($exist)
    {
        Write-Host "File is already downloaded" -ForegroundColor Green
    }
    else
    {
        Write-Host "File is already downloaded but SHA1 is not matched (${hash})" -ForegroundColor Yellow
    }
}

if (!$exist)
{
    Write-Host "Download ${file} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${file}"
}

if ($path -eq "")
{
    Write-Host "No file to download" -ForegroundColor Red
    exit
}

tar -xvf "${path}"
$outputDir = Join-Path $current "${baseName}"

if (Test-Path(${installDir}))
{
    Remove-Item $installDir -Force -Recurse | Out-Null
}
New-ITem -Type Directory ${installDir} -Force | Out-Null
Move-Item "${outputDir}/*" "${installDir}" -Force | Out-Null
Remove-Item $outputDir -Force -Recurse | Out-Null