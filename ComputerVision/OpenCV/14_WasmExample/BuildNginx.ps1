$current = $PSScriptRoot

$file = "nginx-1.26.1.tar.gz"
$url = "https://nginx.org/download/${file}"
$sha1 = "A73998570100134004D665E81783B2A2FF808BCD"

$exist = Test-Path(${file})
if ($exist)
{
    $hash = (Get-FileHash ${file} -Algorithm SHA1).hash
    $exist = $sha1 -eq $hash
    if ($exist)
    {
        Write-Host "Model file is already downloaded" -ForegroundColor Green
    }
    else
    {
        Write-Host "Model file is already downloaded but SHA1 is not matched" -ForegroundColor Yellow
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
}