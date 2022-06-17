$currentDir = $PSScriptRoot
$sourceDir = Join-Path $currentDir sources

$archs = @(
    "x64",
    "Win32"
)

foreach ($arch in $archs)
{
    $spdlogSourceDir = Join-Path $sourceDir spdlog
    $spdlogBuildDir = Join-Path $spdlogSourceDir build | `
                        Join-Path -ChildPath $arch
    $spdlogInstallDir = Join-Path $spdlogSourceDir installs | `
                          Join-Path -ChildPath $arch
    
    New-Item -Type Directory $spdlogBuildDir -Force | Out-Null
    New-Item -Type Directory  $spdlogInstallDir -Force | Out-Null
    
    Push-Location $spdlogBuildDir
    cmake -G "Visual Studio 17 2022" -A $arch `
          -D CMAKE_BUILD_TYPE=Release `
          -D CMAKE_INSTALL_PREFIX="${spdlogInstallDir}" `
          "${spdlogSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location
}