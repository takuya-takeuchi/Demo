$current = Get-Location
$configuration = "Release"
if ($IsWindows)
{
    $taget_dir = "build/win"
    New-Item -ItemType Directory $taget_dir -Force > $Null
    
    Set-Location $taget_dir
    cmake "${current}"
}
else
{
    Write-Host "Not support this platform" -ForegroundColor Red
}

cmake --build . --config ${configuration}

Set-Location $current