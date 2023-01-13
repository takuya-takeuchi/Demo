$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$opencvDir = Join-Path $installDir opencv | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake | `
             Join-Path -ChildPath opencv4

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
$env:OpenCV_DIR=${opencvDir}
cmake -D CMAKE_INSTALL_PREFIX=$installDir `
      -D OpenCV=${opencvDir} `
      $sourceDir
cmake --build . --config Release
Pop-Location
