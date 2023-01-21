$current = $PSScriptRoot
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath natives
New-Item -Type Directory $buildDir -Force | Out-Null

Push-Location $buildDir
cmake ${current}
cmake --build . --config Release
Pop-Location