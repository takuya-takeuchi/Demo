$currentDir = $PSScriptRoot
$sourceDir = Join-Path $currentDir sources

$archs = @(
    "x64",
    "Win32"
)
$VisualStudioVersion = "Visual Studio 17 2022"

foreach ($arch in $archs)
{
    $spdlogSourceDir = Join-Path $sourceDir spdlog
    $spdlogBuildDir = Join-Path $spdlogSourceDir build | `
                      Join-Path -ChildPath $arch
    $spdlogInstallDir = Join-Path $spdlogSourceDir installs | `
                        Join-Path -ChildPath $arch
    
    New-Item -Type Directory $spdlogBuildDir -Force | Out-Null
    New-Item -Type Directory $spdlogInstallDir -Force | Out-Null
    
    Push-Location $spdlogBuildDir
    cmake -G "${VisualStudioVersion}" -A $arch `
          -D CMAKE_BUILD_TYPE=Release `
          -D CMAKE_INSTALL_PREFIX="${spdlogInstallDir}" `
          "${spdlogSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location

    $spdlog_setupSourceDir = Join-Path $sourceDir spdlog_setup
    $spdlog_setupBuildDir = Join-Path $spdlog_setupSourceDir build | `
                            Join-Path -ChildPath $arch
    $spdlog_setupInstallDir = Join-Path $spdlog_setupSourceDir installs | `
                              Join-Path -ChildPath $arch
    
    New-Item -Type Directory $spdlog_setupBuildDir -Force | Out-Null
    New-Item -Type Directory $spdlog_setupInstallDir -Force | Out-Null
    
    Push-Location $spdlog_setupBuildDir
    cmake -G "Visual Studio 17 2022" -A $arch `
          -D CMAKE_INSTALL_PREFIX="${spdlog_setupInstallDir}" `
          -D CMAKE_PREFIX_PATH="${spdlogInstallDir}" `
          "${spdlog_setupSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location
}