#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
)

$current = $PSScriptRoot

$configuration = "Release"

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
else
{
    Write-Host "This platfor is not supported"
}

$visualStudioVersions = @{
    "v143"="Visual Studio 17 2022"
    "v142"="Visual Studio 17 2022"
    "v141"="Visual Studio 17 2022"
}

foreach ($key in $h.Keys) {
	Write-Host $key
	Write-Host $h[$key]
}
foreach ($key in $visualStudioVersions.Keys)
{
    $visualStudio = $visualStudioVersions[$key]
    Write-Host "Build by ${visualStudio} by using ${key} toolset" -ForegroundColor Blue
    
    $sourceDir = $current
    $buildDir = Join-Path $current build | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath program | `
                Join-Path -ChildPath $key
    $installDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program | `
                  Join-Path -ChildPath $key

    New-Item -Type Directory $buildDir -Force | Out-Null
    New-Item -Type Directory $installDir -Force | Out-Null

    Push-Location $buildDir
    cmake -G "${visualStudio}" -A x64 -T ${key},host=x64 `
          -D CMAKE_BUILD_TYPE=${configuration} `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
    cmake --build . --config ${configuration} --target install
    Pop-Location
}