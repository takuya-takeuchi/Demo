$currentDir = $PSScriptRoot
$sourceDir = Join-Path $currentDir sources

$archs = @(
    "x64",
    "Win32"
)

$MsBuildVersion = "2022"
$VisualStudioVersion = "Visual Studio 17 2022"

foreach ($arch in $archs)
{
    $googletestSourceDir = Join-Path $sourceDir googletest
    $googletestBuildDir = Join-Path $googletestSourceDir build | `
                      Join-Path -ChildPath $arch
    $googletestInstallDir = Join-Path $googletestSourceDir installs | `
                        Join-Path -ChildPath $arch
    
    New-Item -Type Directory $googletestBuildDir -Force | Out-Null
    New-Item -Type Directory $googletestInstallDir -Force | Out-Null
    
    Push-Location $googletestBuildDir
    cmake -G "${VisualStudioVersion}" -A $arch `
          -D CMAKE_BUILD_TYPE=Release `
          -D CMAKE_INSTALL_PREFIX="${googletestInstallDir}" `
          "${googletestSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location
}

$msbuild = & "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe | select-object  | Where-Object { $_.Contains("${MsBuildVersion}") }
if (!(Test-Path("${msbuild}")))
{
    Write-Host "msbuild.exe of Visual Studio ${MsBuildVersion} is missing" -ForegroundColor Red
    exit
}

$archs = @(
    "x64",
    "x86"
)

foreach ($arch in $archs)
{
    Push-Location $currentDir
    & "${msbuild}" Demo.sln /t:clean /p:Configuration=Release /p:Platform="${arch}"
    & "${msbuild}" Demo.sln /t:build /p:Configuration=Release /p:Platform="${arch}"
    Pop-Location
}