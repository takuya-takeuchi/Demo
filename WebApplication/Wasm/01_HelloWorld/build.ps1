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

$program = "helloworld"
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

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    emcmake cmake $sourceDir
}
elseif ($global:IsMacOS)
{
    emcmake cmake $sourceDir
    emmake make && emmake make package
}
elseif ($global:IsLinux)
{
    emcmake cmake $sourceDir
    emmake make && emmake make package
    Copy-Item "${buildDir}/${program}*" "${installDir}" -Force
}
Pop-Location

# # run
# if ($global:IsWindows)
# {
#     $programDir = Join-Path $installDir bin
#     $library_type = "x64-windows"
#     Copy-Item "${env:VCPKG_ROOT_DIR}\installed\${library_type}\bin\rdkafka.dll" "${programDir}" -Force
#     Copy-Item "${env:VCPKG_ROOT_DIR}\installed\${library_type}\bin\lz4.dll" "${programDir}" -Force
#     $program = Join-Path $programDir Test.exe
# }
# elseif ($global:IsMacOS)
# {
#     $programDir = Join-Path $current build | `
#                   Join-Path -ChildPath $os | `
#                   Join-Path -ChildPath program
#     $program = Join-Path $programDir Test
# }
# elseif ($global:IsLinux)
# {
#     $programDir = Join-Path $current build | `
#                   Join-Path -ChildPath $os | `
#                   Join-Path -ChildPath program
#     $program = Join-Path $programDir Test
# }