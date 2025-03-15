#$current = $PSScriptRoot
$current = Get-Location

if ($global:IsWindows)
{
    $file = "nginx-1.26.1.zip"
    $expandedDir = Join-Path $current "nginx-1.26.1"
    $url = "https://nginx.org/download/${file}"
    $sha1 = "AF2E1D90BCD7754ACBBDDEFB88159557C1C2BEC3"
}
elseif ($global:IsMacOS)
{
    $file = "nginx-1.26.1.tar.gz"
    $url = "https://nginx.org/download/${file}"
    $sha1 = "A73998570100134004D665E81783B2A2FF808BCD"
}
elseif ($global:IsLinux)
{
    $file = "nginx-1.26.1.tar.gz"
    $url = "https://nginx.org/download/${file}"
    $sha1 = "A73998570100134004D665E81783B2A2FF808BCD"
}

$exist = Test-Path(${file})
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
        Write-Host "File is already downloaded but SHA1 is not matched" -ForegroundColor Yellow
    }
}

if (!$exist)
{
    Write-Host "Download ${file} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${file}"
}

$sourceDir = Join-Path $current nginx | `
             Join-Path -ChildPath src
$installDir = Join-Path $current nginx
if ($global:IsWindows)
{
    Expand-Archive -Path $file -DestinationPath $current -Force
    
    $exist = Test-Path(${installDir})
    if ($exist)
    {
        Remove-Item ${installDir} -Force -Recurse | Out-Null
    }

    Move-Item $expandedDir $installDir -Force | Out-Null
}
elseif ($global:IsMacOS)
{
    New-Item -Type Directory $sourceDir -Force | Out-Null
    tar -xvzf $file -C $sourceDir --strip-components 1

    Push-Location $sourceDir | Out-Null
    ./configure --prefix=$installDir
    make
    make install
    Pop-Location
}
elseif ($global:IsLinux)
{
    New-Item -Type Directory $sourceDir -Force | Out-Null
    tar -xvzf $file -C $sourceDir --strip-components 1

    Push-Location $sourceDir | Out-Null
    ./configure --prefix=$installDir --without-http_rewrite_module --without-http_gzip_module
    make
    make install
    Pop-Location
}