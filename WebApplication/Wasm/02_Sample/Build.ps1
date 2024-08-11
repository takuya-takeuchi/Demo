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

$target = "opencv4"
$shared = "static"
$emSdkVersion = "3.1.64"

# build
$buildDirName = Join-Path "build" $os
$installDirName = Join-Path "install" $os
$buildDir = $buildDirName
$installDir = $installDirName

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

$currentDirName = Split-Path -Path $current -Leaf
$rootDir = Split-Path $current -Parent

if ($global:IsWindows)
{
    $buildDirName = $buildDirName.Replace("`\", "/")
    $installDirName = $installDirName.Replace("`\", "/")
}

$sourceDir = "/project/${currentDirName}"
$buildCommand = "emcmake cmake -B ${buildDirName} -D CMAKE_INSTALL_PREFIX=${installDirName} -D CMAKE_BUILD_TYPE=${Configuration} ${sourceDir}"
$installCommand = "cmake --build ${buildDirName} --config $Configuration --target install"
docker run --rm --workdir "${sourceDir}" `
           -v "${rootDir}:/project" "emscripten/emsdk:${emSdkVersion}" sh -c "${buildCommand} && ${installCommand}"