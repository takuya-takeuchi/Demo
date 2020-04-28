$current = Get-Location
$build_dir = Join-Path $PSScriptRoot build
New-Item ${build_dir} -ItemType Directory -Force | Out-Null
Set-Location ${build_dir}
cmake ..
cmake --build . --config Release
cmake --build . --config Debug