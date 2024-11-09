#***************************************
#Arguments
#%1: Build Configuration (Release/Debug/RelWithDebInfo/MinSizeRel)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration
)

# get os name
if ($global:IsWindows)
{
    Write-Host "Windows is not supported!!" -ForegroundColor Red
    exit
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$configuration = $Configuration

$ConfigurationArray =
@(
   "Debug",
   "Release"
)

if ($ConfigurationArray.Contains($configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Error: Specify Configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot

$buildTarget = "bloaty"

$sourceDir = Join-Path $current $buildTarget
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

# restore
git submodule update --init --recursive .

Push-Location $buildDir

switch ($os)
{
    "linux"
    {
        cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D TARGET_ARCHITECTURES="${architecture}"  `
              -D BUILD_TESTING="OFF" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
    "osx"
    {
        cmake -G Ninja `
              -D CMAKE_INSTALL_PREFIX=${installDir} `
              -D CMAKE_BUILD_TYPE=${configuration} `
              -D BUILD_TESTING="OFF" `
              $sourceDir
        cmake --build . --config ${configuration} --target install
    }
}
Pop-Location

# run
switch ($os)
{
    "linux"
    {
        $installDir = Join-Path ${installDir} "bin"
        $program = Join-Path $installDir ${buildTarget}
        pushd ${installDir}
        $program = Join-Path $installDir ${buildTarget}
        & "${program}" --version
        popd
    }
    "osx"
    {
        $installDir = Join-Path ${installDir} "bin"
        $program = Join-Path $installDir ${buildTarget}
        pushd ${installDir}
        $program = Join-Path $installDir ${buildTarget}
        & "${program}" --version
        popd
    }
}