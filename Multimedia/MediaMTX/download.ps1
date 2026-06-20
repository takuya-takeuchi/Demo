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

$target = "mediamtx"
$tag = $config.mediamtx.version
$baseUrl = "https://github.com/bluenviron/mediamtx/releases/download"

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $baseName = "mediamtx_${tag}_windows_amd64"
    $file = "${baseName}.zip"
    $sha256 = $config.mediamtx.sha256.win
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $baseName = "mediamtx_${tag}_darwin_arm64"
    $file = "${baseName}.tar.gz"
    $sha256 = $config.mediamtx.sha256.osx
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $baseName = "mediamtx_${tag}_linux_amd64"
    $file = "${baseName}.tar.gz"
    $sha256 = $config.mediamtx.sha256.linux
}

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $os

$path = ""
$url = "${baseUrl}/${tag}/${file}"
$sha256 = $entry.Value;

$path = Join-Path $current $file
$exist = Test-Path(${path})
if ($exist)
{
    $hash = (Get-FileHash ${file} -Algorithm SHA256).hash
    $exist = $sha256 -eq $hash
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

$outputDir = Join-Path $current "${baseName}"
New-Item -Type Directory ${outputDir} -Force | Out-Null
tar -xzvf "${path}" -C "${outputDir}"

if (Test-Path(${installDir}))
{
    Remove-Item $installDir -Force -Recurse | Out-Null
}
New-Item -Type Directory ${installDir} -Force | Out-Null
Move-Item "${outputDir}/*" "${installDir}" -Force | Out-Null
Remove-Item $outputDir -Force -Recurse | Out-Null