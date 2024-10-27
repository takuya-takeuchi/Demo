#***************************************
#Arguments
#%1: Version  (e.g. 3.3.2)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Version
)

$version = $Version

$basename = "openssl-${version}"
$basename2 = "openssl-${version}"

# if version 1.x
# https://github.com/openssl/openssl/releases/download/openssl_1_1_1w/openssl-1.1.1w.tar.gz.sha1
if (${version}.StartsWith("1."))
{
    $basename2 = "OpenSSL_" + ${version}.Replace(".", "_")
}

$sourceFile = "${basename}.tar.gz"
$hashFile = "${sourceFile}.sha1"
$sourceUrl = "https://github.com/openssl/openssl/releases/download/${basename2}/${sourceFile}"
$hashUrl = "https://github.com/openssl/openssl/releases/download/${basename2}/${hashFile}"

$current = $PSScriptRoot
$sourceFile = Join-Path $current $sourceFile
$hashFile = Join-Path $current $hashFile

$ProgressPreference = 'SilentlyContinue'
Write-Host "Download hash file from ${hashUrl}" -ForegroundColor Blue
Invoke-WebRequest "${hashUrl}" -OutFile ${hashfile} -SkipHttpErrorCheck
$sha1 = (Get-Content ${hashfile}).ToUpper().Trim()
Remove-Item ${hashfile}
$ProgressPreference = 'Continue'

if ($sha1 -eq "")
{
    Write-Host "${hashUrl} is missing" -ForegroundColor Red
    exit
}

$exist = Test-Path(${sourceFile})
if ($exist)
{
    $hash = (Get-FileHash ${sourceFile} -Algorithm SHA1).hash
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
    Write-Host "Download ${sourceFile} from ${sourceUrl}" -ForegroundColor Blue
    Invoke-WebRequest "${sourceUrl}" -OutFile "${sourceFile}"
}

$sourceDir = Join-Path $current openssl
if (!(Test-Path($sourceDir)))
{
    New-Item -Type Directory $sourceDir -Force | Out-Null
}

$sourceDir = Join-Path $sourceDir "${version}"
if (Test-Path($sourceDir))
{
    Remove-Item $sourceDir -Force -Recurse | Out-Null
}

New-Item -Type Directory $sourceDir -Force | Out-Null

tar -xzvf "${sourceFile}"
$outputDir = Join-Path $current "${basename}"
Move-Item "${outputDir}/*" "${sourceDir}" -Force

Start-Sleep -Seconds 5
Remove-Item $outputDir -Force -Recurse | Out-Null