#***************************************
#Arguments
#%1: Architecture (x86_64/arm64)
#%2: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Architecture,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Configuration
)


$os = "osx"
$configuration = $Configuration
$architecture = $Architecture

$ConfigurationArray =
@(
   "Debug",
   "Release"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64"
)

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray.Keys -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray.Keys -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot

$buildTarget = "ObjectiveLuhnWrapper"
$wrappedLibrary = "ObjectiveLuhn"

# build
$sourceDir = Join-Path $current ${buildTarget}
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture | `
            Join-Path -ChildPath $buildTarget
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
            Join-Path -ChildPath $architecture | `
              Join-Path -ChildPath $buildTarget

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

$wrappedLibraryDir = Join-Path $current install | `
                     Join-Path -ChildPath "${os}" | `
                     Join-Path -ChildPath "${architecture}" | `
                     Join-Path -ChildPath "${wrappedLibrary}"

cmake -D CMAKE_BUILD_TYPE=$Configuration `
      -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D CMAKE_SYSTEM_PROCESSOR="${architecture}" `
      -D CMAKE_OSX_ARCHITECTURES="${architecture}" `
      -D CMAKE_MACOSX_RPATH="ON" `
      -D LUHN_SDK_DIR="${wrappedLibraryDir}" `
      "${sourceDir}"
cmake --build . --config ${Configuration} --target install

# install_name_tool -delete_rpath "/System/Volumes/Preboot/Cryptexes/OS@rpath/libLuhnC.dylib" "${installDir}/lib/libLuhnC.dylib"
# install_name_tool -add_rpath @loader_path/  "${installDir}/lib/libLuhnC.dylib"
# install_name_tool -id @rpath/libLuhnC.dylib  "${installDir}/lib/libLuhnC.dylib"
Pop-Location