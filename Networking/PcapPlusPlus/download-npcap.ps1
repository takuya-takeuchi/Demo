$target = "npcap-sdk"
$version = "1.15"

$current = $PSScriptRoot

# get os name, source file name and hash value
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

$sha1 = "1E15A1349A58B676200111E35C1BE5537C38A763"
$filename = "npcap-sdk-${version}.zip"
$url = "https://npcap.com/dist/${filename}"

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

$sourceDir = Join-Path $current $target | `
             Join-Path -ChildPath "${version}"
$exist = Test-Path(${sourceDir})
if (!$exist)
{
    $targetDir = Join-Path $current $target
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
