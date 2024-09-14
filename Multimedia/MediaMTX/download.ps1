$current = $PSScriptRoot

$target = "MediaMTX"
$baseUrl = "https://github.com/bluenviron/mediamtx/releases/download"
$tag = "v1.9.0"

$urls = @{
    "mediamtx_${tag}_windows_amd64.zip"="1F1AFDD922294E119B489D4AE7B5AE2483C055A4";
    "mediamtx_${tag}_linux_amd64.tar.gz"="5B60F775D1B440ACC6A003EDCB63D5816160917C";
    "mediamtx_${tag}_linux_arm64v8.tar.gz"="EE2B258B46F4106D175BC86A84DD0C46756C8DAA";
}

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $baseName = "mediamtx_${tag}_windows_amd64"
    $key = "${baseName}.zip"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $baseName = "mediamtx_${tag}_linux_arm64v8"
    $key = "${baseName}.tar.gz"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $baseName = "mediamtx_${tag}_linux_amd64"
    $key = "${baseName}.tar.gz"
}

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $os

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