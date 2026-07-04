$current = $PSScriptRoot

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

$configPath = Join-Path $current "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}
$config = Get-Content -Path $configPath | ConvertFrom-Json

$target = "llvm"
$version = $config.llvm.version
$installDir = Join-PathArray -PathElements @($current, "install", $os, $target)
if (Test-Path(${installDir}))
{
    Remove-Item $installDir -Force -Recurse | Out-Null
}
New-Item -Type Directory ${installDir} -Force | Out-Null

# get os name
if ($global:IsWindows)
{
    $baseName = "clang+llvm-${version}-x86_64-pc-windows-msvc"
    $url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-${version}/${baseName}.tar.xz"
    $sha256 = $config.llvm.win.sha256
    $file = Split-Path -Leaf ${url}
}
elseif ($global:IsMacOS)
{
    $baseName = "clang+llvm-${version}-arm64-apple-macos11"
    $url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-${version}/${baseName}.tar.xz"
    $sha256 = $config.llvm.osx.sha256
    $file = Split-Path -Leaf ${url}
}
elseif ($global:IsLinux)
{
    $baseName = "clang+llvm-${version}-x86_64-linux-gnu-ubuntu-18.04"
    $url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-${version}/${baseName}.tar.xz"
    $sha256 = $config.llvm.linux.sha256
    $file = Split-Path -Leaf ${url}
}
else
{
    Write-Host "This platform is not supported" -ForegroundColor Red
    exit
}

$path = ""
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
        Write-Host "File is already downloaded but SHA256 is not matched (${hash})" -ForegroundColor Yellow
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