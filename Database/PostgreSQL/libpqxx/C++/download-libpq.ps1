$current = $PSScriptRoot

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

$target = "libpq"
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target
if (Test-Path(${installDir}))
{
    Remove-Item $installDir -Force -Recurse | Out-Null
}
New-Item -Type Directory ${installDir} -Force | Out-Null

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $url = $config.libpq.win.url
    $sha256 = $config.libpq.win.sha256
    $file = "${baseName}.zip"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $url = $config.libpq.osx.url
    $sha256 = $config.libpq.osx.sha256
}
elseif ($global:IsLinux)
{
    $tmpDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath tmp
    New-Item -Type Directory ${tmpDir} -Force | Out-Null

    $hasApt = Get-Command apt -ErrorAction SilentlyContinue
    $hasDnf = Get-Command dnf -ErrorAction SilentlyContinue
    $hasYum = Get-Command yum -ErrorAction SilentlyContinue

    Push-Location $tmpDir

    if ($hasApt)
    {
        apt download libpq-dev
        $path = Get-ChildItem -Path "${tmpDir}" -Filter "*.deb" | Select-Object -First 1
    }
    elseif ($hasDnf)
    {
        dnf download libpq-devel
        $path = Get-ChildItem -Path "${tmpDir}" -Filter "*.rpm" | Select-Object -First 1
    }
    elseif ($hasYum)
    {
        yum install libpq-devel --downloadonly
        $path = Get-ChildItem -Path "${tmpDir}" -Filter "*.rpm" | Select-Object -First 1
    }
    else
    {
        Write-Host "This distribution is not supported" -ForegroundColor Red
        exit
    }

    Pop-Location
}
else
{
    Write-Host "This platform is not supported" -ForegroundColor Red
    exit
}

if (!($global:IsLinux))
{
    $path = Join-Path $current $file
    $url = "${baseUrl}/${tag}/${file}"
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
}

if ($path -eq "")
{
    Write-Host "No file to download" -ForegroundColor Red
    exit
}

# get os name
if ($global:IsLinux)
{
    $hasApt = Get-Command apt -ErrorAction SilentlyContinue
    $hasDnf = Get-Command dnf -ErrorAction SilentlyContinue
    $hasYum = Get-Command yum -ErrorAction SilentlyContinue

    Push-Location $tmpDir

    if ($hasApt)
    {
        ar x "${path}"
        tar -xf "${tmpDir}/data.tar.zst" -C "${installDir}" --strip-components=2
    }
    elseif ($hasDnf -or $hasYum)
    {
        rpm2cpio "${path}" | cpio -idmv
    }

    Pop-Location
    Remove-Item $tmpDir -Force -Recurse | Out-Null
}
else
{
    Extract-Archive -Path "${path}" -DestinationPath "${installDir}" -Force
}