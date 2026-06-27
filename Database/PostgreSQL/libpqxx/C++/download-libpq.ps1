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
    $file = "postgresql-${os}-binaries.zip"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $url = $config.libpq.osx.url
    $sha256 = $config.libpq.osx.sha256
    $file = "postgresql-${os}-binaries.zip"
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

    $version = $config.libpq.linux.version
    $majorVersion = $version.Split(".")[0]

    if ($hasApt)
    {
        $osReleaseContent = Get-Content -Path /etc/os-release -Raw
        $osInfo = ConvertFrom-StringData -StringData $osReleaseContent
        $codename = $osInfo.VERSION_CODENAME
        $versionId = $osInfo.VERSION_ID.Replace("`"", "")

        $urls = @(
            "https://ftp.postgresql.org/pub/repos/apt/pool/main/p/postgresql-${majorVersion}/libpq-dev_${version}-1.pgdg${versionId}+1_amd64.deb",
            "https://ftp.postgresql.org/pub/repos/apt/pool/main/p/postgresql-${majorVersion}/postgresql-server-dev-${majorVersion}_${version}-1.pgdg${versionId}+1_amd64.deb"
        )

        foreach ($url in $urls)
        {
            $deb = "${tmpDir}/$(Split-Path -Leaf ${url})"
            Write-Host "Download ${url} to ${deb}" -ForegroundColor Blue
            Invoke-WebRequest "${url}" -OutFile "${deb}"
        }
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
        $debs = Get-ChildItem -Path "${tmpDir}" -Filter "*.deb"
        foreach ($deb in $debs)
        {
            ar x "${deb}"
            tar -xf "data.tar.xz" -C "${installDir}" --strip-components=1
        }
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
    # Do NOT use Eapand-Archive to extract postgresql zip because it could have symbolic links and Expand-Archive does not support symbolic links
    #Expand-Archive -Path "${path}" -DestinationPath "${installDir}" -Force
    unzip -o "${path}" -d "${installDir}"
}