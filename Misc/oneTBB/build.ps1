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

$target = "oneTBB"
# For onTBB, Static library is highly discouraged and such configuration is not supported.
$shared = "dynamic"
$sharedFlag = "ON"
$tbbVerifyDependencySignature = "OFF"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

git submodule update --init --recursive .

Push-Location $buildDir
if ($global:IsWindows)
{
    # On not English locale system, C4819 warning will be treated as C2220 error.
    # Passs /wd4819 to avoid this behavior.
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D TBB_VERIFY_DEPENDENCY_SIGNATURE=${tbbVerifyDependencySignature} `
          -D CMAKE_CXX_FLAGS="/wd4819" `
          -D CMAKE_C_FLAGS="/wd4819" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D TBB_VERIFY_DEPENDENCY_SIGNATURE=${tbbVerifyDependencySignature} `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D TBB_VERIFY_DEPENDENCY_SIGNATURE=${tbbVerifyDependencySignature} `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location