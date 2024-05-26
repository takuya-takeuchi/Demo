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
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "f3d"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $Configuration

$vtkInstallDir = Join-Path $current install | `
                 Join-Path -ChildPath $os | `
                 Join-Path -ChildPath VTK | `
                 Join-Path -ChildPath $Configuration

$glfwInstallDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath glfw | `
                  Join-Path -ChildPath $Configuration | `
                  Join-Path -ChildPath lib | `
                  Join-Path -ChildPath cmake | `
                  Join-Path -ChildPath glfw3

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir | Out-Null

if ($global:IsWindows)
{
    # to build testing
    Push-Location $sourceDir | Out-Null
    git lfs pull
    Pop-Location

    $env:glfw3_DIR=${glfwInstallDir}
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH=$vtkInstallDir `
          -D BUILD_TESTING=ON `
          $sourceDir
}
elseif ($global:IsMacOS)
{
}
elseif ($global:IsLinux)
{
}
cmake --build . --config $Configuration --target install
Pop-Location