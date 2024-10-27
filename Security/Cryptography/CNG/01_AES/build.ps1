#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration
)

$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
else
{
    Write-Host "[Error] This platform is not supported" -ForegroundColor Red
    exit
}

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D CMAKE_PREFIX_PATH="${targetDir}" `
      $sourceDir
cmake --build . --config ${Configuration} --target install
Pop-Location