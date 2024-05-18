#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#%2: Build System that built Tensorflow (cmake or bazel)
#%3: Tensorflow Version (v2.5.3)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration,
   
   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $buildSystem,
   
   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $tensorflowVersion
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

$target = "tensorflow"
$lang = "c"

# build
$rootDir = Split-Path $current -Parent
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program |
            Join-Path -ChildPath $buildSystem
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os |
              Join-Path -ChildPath $buildSystem
$installBinaryDir = Join-Path $installDir bin

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $targetInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target | `
                        Join-Path -ChildPath $lang | `
                        Join-Path -ChildPath $buildSystem |
                        Join-Path -ChildPath $Configuration |
                        Join-Path -ChildPath $tensorflowVersion
    if (!(Test-Path(${targetInstallDir})))
    {
        Write-Host "${targetInstallDir} is missing" -ForegroundColor Red
        return
    }
    
    $includeDir = Join-Path $targetInstallDir "include"
    $libDir = Join-Path $targetInstallDir "lib"
    $libs = Get-ChildItem -Path $libDir -Recurse -File | Where-Object { @(".lib") -contains $_.Extension }
    $libs = ($libs | ForEach-Object { $_.FullName }) -join ";"
    $bins = Get-ChildItem -Path $libDir -Recurse -File | Where-Object { @(".dll") -contains $_.Extension }
    $bins = ($bins | ForEach-Object { $_.FullName }) -join ";"

    # cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
    #       -D TFLITE_ROOT="${targetInstallDir}" `
    #       $sourceDir
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D TFLITE_BINS="${bins}" `
          -D TFLITE_LIBS="${libs}" `
          -D TFLITE_INCLUDE_DIR="${includeDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
}
elseif ($global:IsLinux)
{
}
cmake --build . --config ${Configuration} --target install
Pop-Location