#***************************************
#Arguments
#   Configuration: Release/Debug
#***************************************
Param([Parameter(
      Mandatory=$True,
      Position = 1
      )][string]
      $Configuration
)

$root = $PSScriptRoot

if ($global:IsWindows)
{
    $buildDir = Join-Path $root build | `
                Join-Path -ChildPath windows
    $installDir = Join-Path $root install | `
                  Join-Path -ChildPath windows
}
else
{
   Write-Host "Error: This plaform is not support" -ForegroundColor Red
   exit -1
}

New-Item -Type Directory $buildDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 `
          -D CMAKE_BUILD_TYPE:STRING=${Configuration} `
          -D CMAKE_INSTALL_PREFIX:PATH="${installDir}" `
          "${root}"
}
cmake --build . --config ${Configuration} --target install
Pop-Location