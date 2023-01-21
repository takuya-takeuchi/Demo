$current = $PSScriptRoot
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath natives
New-Item -Type Directory $buildDir -Force | Out-Null

Push-Location $buildDir

if ($global:IsWindows)
{
    cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 ${current}
}
else
{
    cmake ${current}
}

cmake --build . --config Release
Pop-Location