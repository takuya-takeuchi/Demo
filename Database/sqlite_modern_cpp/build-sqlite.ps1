$target = "sqlite"
$version = "3.45.1"
$filename = "sqlite-amalgamation-3450100.zip"
$url = "https://www.sqlite.org/2024/${filename}"

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

$filepath = Join-Path $current $filename
if (!(Test-Path("${filepath}")))
{
    Write-Host "Download ${target} source code from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${filename}"
}

$sourceDir = Join-Path $current $target | `
             Join-Path -ChildPath $version
New-Item -Type Directory $sourceDir -Force | Out-Null
Expand-Archive -Path "${filename}" -DestinationPath $current
$basename = (Get-Item $filename).BaseName
Move-Item "$basename/*" $sourceDir -Force | Out-Null

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
Copy-Item "$sourceDir/*" $installDir -Force  | Out-Null