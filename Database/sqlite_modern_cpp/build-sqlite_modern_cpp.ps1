$target = "sqlite_modern_cpp"
$url = "https://github.com/SqliteModernCpp/sqlite_modern_cpp"
$branch = "v3.2"

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

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $installDir -Force | Out-Null

if (!(Test-Path($target)))
{
    git clone $url
}

Push-Location $target
git checkout $branch
Copy-Item "hdr/*" -Force -Recurse $installDir
Pop-Location