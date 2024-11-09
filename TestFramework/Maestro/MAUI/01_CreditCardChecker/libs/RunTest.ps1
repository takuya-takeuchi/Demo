#***************************************
#Arguments
#%1: Target (windows/linux/osx)
#%2: Architecture (x86_64/arm64)
#%3: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Target,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Architecture,

   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $Configuration
)

# root config
$macosDeplolymentTarget = "11.0"

$target = $Target
$architecture = $Architecture
$configuration = $Configuration

$TargetArray =
@(
   "windows",
   "linux",
   "osx"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64"
)

$ConfigurationArray =
@(
   "Debug",
   "Release"
)

if ($TargetArray.Contains($target) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Error: Specify Target [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot
$libDir = $current
$installLibDir = Join-Path $libDir install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $architecture | `
              Join-Path -ChildPath share | `
              Join-Path -ChildPath cmake | `
              Join-Path -ChildPath Luhn

$current = Join-Path $current test
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $architecture | `
            Join-Path -ChildPath test
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $architecture | `
              Join-Path -ChildPath test

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

git submodule update --init --recursive .

switch ($target)
{
   "windows"
   {
      $googletestSourceDir = Join-Path $sourceDir googletest
      $googletestBuildDir = Join-Path $sourceDir build | `
                            Join-Path -ChildPath $target | `
                            Join-Path -ChildPath $architecture | `
                            Join-Path -ChildPath googletest
      $googletestInstallDir = Join-Path $sourceDir install | `
                              Join-Path -ChildPath $target | `
                              Join-Path -ChildPath $architecture | `
                              Join-Path -ChildPath googletest
      
      New-Item -Type Directory $googletestBuildDir -Force | Out-Null
      New-Item -Type Directory $googletestInstallDir -Force | Out-Null
      
      Push-Location $googletestBuildDir
      cmake -D CMAKE_BUILD_TYPE=${configuration} `
            -D BUILD_SHARED_LIBS=True `
            -D CMAKE_INSTALL_PREFIX="${googletestInstallDir}" `
            "${googletestSourceDir}"
      cmake --build . --config ${configuration} --target install
      Pop-Location

      Push-Location $buildDir
      cmake -D CMAKE_BUILD_TYPE=${configuration} `
            -D CMAKE_INSTALL_PREFIX=${installDir} `
            -D CMAKE_PREFIX_PATH="${googletestInstallDir}\lib\cmake\GTest;${installLibDir}" `
            $sourceDir
      cmake --build . --config ${configuration} --target install
      Pop-Location

      Copy-Item "${googletestInstallDir}\*" "${installDir}" -Force -Recurse
   }
   "linux"
   {
      
   }
   "osx"
   {
      
   }
}
Pop-Location