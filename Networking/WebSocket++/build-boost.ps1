#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration
)

$target = "boost"
$version = "1.86.0"
$shared = "static"

$current = $PSScriptRoot

$filenameVersion = ${version}.Replace(".", "_")

# get os name, source file name and hash value
if ($global:IsWindows)
{
    $os = "win"
    $filename = "boost_${filenameVersion}.zip"
    $sha1 = "B7190A4526F83428074811DB4431C9D7D67C85D1"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $filename = "boost_${filenameVersion}.tar.gz"
    $sha1 = "C248C60E2CE74C77CAE3F678EDE2F65491C4D815"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $filename = "boost_${filenameVersion}.tar.gz"
    $sha1 = "C248C60E2CE74C77CAE3F678EDE2F65491C4D815"
}

$url = "https://boostorg.jfrog.io/artifactory/main/release/${version}/source/${filename}"

$exist = Test-Path(${filename})
if ($exist)
{
    $hash = (Get-FileHash ${filename} -Algorithm SHA1).hash
    $exist = $sha1 -eq $hash
    if ($exist)
    {
        Write-Host "Source archive file is already downloaded" -ForegroundColor Green
    }
    else
    {
        Write-Host "Source archive file is already downloaded but SHA1 is not matched (${hash})" -ForegroundColor Yellow
    }
}

if (!$exist)
{
    Write-Host "Download ${filename} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${filename}"
}

$filenameVersion = $version -replace "\.", "_"
$sourceDir = Join-Path $current boost | `
             Join-Path -ChildPath "boost_${filenameVersion}"
$exist = Test-Path(${sourceDir})
if (!$exist)
{
    $targetDir = Join-Path $current boost
    New-Item -Type Directory $targetDir -Force | Out-Null
    if (!(Test-Path("${sourceDir}")))
    {    
        if ($global:IsWindows)
        {
            Expand-Archive -Path "${filename}" -DestinationPath $targetDir
        }
        else
        {
            tar -xzf "${filename}" -C "${targetDir}"
        }
    }
}
else
{
    Write-Host "Source directory is already present" -ForegroundColor Green
}

$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared
              
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir

# https://www.boost.org/doc/libs/develop/doc/html/boost_asio/using.html
# build option to use asio is `--with-system --with-thread --with-date_time --with-regex --with-serialization`
if ($global:IsWindows)
{
    $bootstrap = Join-Path $sourceDir bootstrap.bat
    $b2 = Join-Path $sourceDir b2.exe
    & "${bootstrap}"
    & "${b2}" link=$shared `
              address-model=64 `
              install `
              -j2 `
              --prefix=${installDir} `
              --build-dir=${buildDir} `
              --with-system --with-thread --with-date_time --with-regex --with-serialization
}
else
{
    $bootstrap = Join-Path $sourceDir bootstrap.sh
    $b2 = Join-Path $sourceDir b2
    & bash "${bootstrap}"
    & "${b2}" link=$shared `
              address-model=64 `
              install `
              -j2 `
              --prefix=${installDir} `
              --build-dir=${buildDir} `
              --with-system --with-thread --with-date_time --with-regex --with-serialization
}
Pop-Location
