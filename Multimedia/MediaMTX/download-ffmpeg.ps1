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

$target = "FFmpeg"
$version = "7.0.2-17"
$majorVersion = "7.0"
$baseUrl = "https://github.com/BtbN/FFmpeg-Builds/releases/download"
$tag = "autobuild-2024-09-14-12-50"
$commitHash = "gf705bc5b73"

$urls = @{
    "ffmpeg-n${version}-${commitHash}-win64-gpl-${majorVersion}.zip"="23E4CA6FF5529F8E3ADA5DE53C6B0D4295E34027";
    "ffmpeg-n${version}-${commitHash}-win64-lgpl-${majorVersion}.zip"="95BCDD2306544C44ECF4ADD30C3E50ACDEAF88A2";
    "ffmpeg-n${version}-${commitHash}-linux64-gpl-${majorVersion}.tar.xz"="B4E3742F1308A21F0CE85A6379996F380763096F";
    "ffmpeg-n${version}-${commitHash}-linux64-lgpl-${majorVersion}.tar.xz"="24943A28C86FF649AFE7567EDF384F725FDF1185";
    "ffmpeg-n${version}-${commitHash}-linuxarm64-gpl-${majorVersion}.tar.xz"="4324BA30D695272A4B2996E74016B5089028DC2A";
    "ffmpeg-n${version}-${commitHash}-linuxarm64-lgpl-${majorVersion}.tar.xz"="C751DD2AF63279209F1672CEB8CDFA09798C92E7";
}

$gplType = $Gpl ? "gpl" : "lgpl"
Write-Host "Download ${gplType} version" -ForegroundColor Yello

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $baseName = "ffmpeg-n${version}-${commitHash}-win64-${gplType}-${majorVersion}"
    $key = "${baseName}.zip"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $baseName = "ffmpeg-n${version}-${commitHash}-linuxarm64-${gplType}-${majorVersion}"
    $key = "${baseName}.tar.xz"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $baseName = "ffmpeg-n${version}-${commitHash}-linux64-${gplType}-${majorVersion}"
    $key = "${baseName}.tar.xz"
}

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $gplType

$path = ""
foreach($entry in $urls.GetEnumerator())
{
    $file = $entry.Key
    if ($file -ne $key)
    {
        continue
    }

    $url = "${baseUrl}/${tag}/${file}"
    $sha1 = $entry.Value;

    $path = Join-Path $current $file
    $exist = Test-Path(${path})
    if ($exist)
    {
        $hash = (Get-FileHash ${file} -Algorithm SHA1).hash
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