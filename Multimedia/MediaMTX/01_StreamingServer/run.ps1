$current = $PSScriptRoot

$target = "MediaMTX"

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $program = "mediamtx.exe"
    $ffmpeg = "ffmpeg.exe"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $program = "mediamtx"
    $ffmpeg = "ffmpeg"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $program = "mediamtx"
    $ffmpeg = "ffmpeg"
}

$rootDir = Split-Path $current -Parent
$installDir = Join-Path $rootDir install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $os
$ffmpeg = Join-Path $rootDir install | `
          Join-Path -ChildPath FFmpeg | `
          Join-Path -ChildPath $os | `
          Join-Path -ChildPath lgpl | `
          Join-Path -ChildPath bin | `
          Join-Path -ChildPath $ffmpeg
$program = Join-Path $installDir $program
if (!(Test-Path(${program})))
{
    Write-Host "'${program}' is missing" -ForegroundColor Red
    exit
}

$env:FFMPEG="${ffmpeg}"
& "${program}" mediamtx.yml