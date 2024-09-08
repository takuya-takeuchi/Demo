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
    $libExt = ".lib"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $libExt = ".a"
}

$trtPath = $env:TENSORRT_PATH
if (!$trtPath)
{
    Write-Host "Environmental Variable 'TENSORRT_PATH' is missing" -ForegroundColor Red
    exit
}

$tensorRTIncludeDir = Join-Path "${trtPath}" include
$tensorRTLibDir = Join-Path "${trtPath}" lib

$tensorRTIncludeDir = $tensorRTIncludeDir -replace '\\', '/' 
$tensorRTLibs = Get-ChildItem -Path $tensorRTLibDir -Recurse -File | Where-Object { @($libExt) -contains $_.Extension }
$tensorRTLibs = ($tensorRTLibs | ForEach-Object { $_.FullName }) -join ";"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

$rootDir = Split-Path $current -Parent
$openCVInstallDir = Join-Path $rootDir install-enable-freetype | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target

Push-Location $buildDir
if ($global:IsWindows)
{
    # $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D TENSORRT_INCLUDE_DIRS="${tensorRTIncludeDir}" `
          -D TENSORRT_LIBRARIES="${tensorRTLibs}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${tensorRTLibs}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

$cudaPath = $env:CUDA_PATH
if (!$cudaPath)
{
    Write-Host "Environmental Variable 'CUDA_PATH' is missing" -ForegroundColor Red
    exit
}

$trtPath = $env:TENSORRT_PATH
if (!$trtPath)
{
    Write-Host "Environmental Variable 'TENSORRT_PATH' is missing" -ForegroundColor Red
    exit
}

$cudaBinDir = Join-Path "${cudaPath}" bin
$tensorRTBinDir = Join-Path "${trtPath}" bin
$tensorRTLibDir = Join-Path "${trtPath}" lib

# get os name
if ($global:IsWindows)
{
    $binaries = @{
        # TensorRT 10
        "nvinfer_10.dll"                  = "${tensorRTLibDir}";
        "nvinfer_plugin_10.dll"           = "${tensorRTLibDir}";
        "nvonnxparser_10.dll"             = "${tensorRTLibDir}";
        "nvinfer_builder_resource_10.dll" = "${tensorRTLibDir}";
        # TensorRT 8
        "nvinfer.dll"                     = "${tensorRTLibDir}";
        "nvinfer_plugin.dll"              = "${tensorRTLibDir}";
        "nvonnxparser.dll"                = "${tensorRTLibDir}";
        "nvparsers.dll"                   = "${tensorRTLibDir}";
        "cudart64_110.dll"                = "${cudaBinDir}"
        "cublas64_11.dll"                 = "${cudaBinDir}"
        "cublasLt64_11.dll"               = "${cudaBinDir}"
    }
}
elseif ($global:IsLinux)
{
}

$installDir = Join-Path $installDir bin
Push-Location $installDir
foreach ($kvp in $binaries.GetEnumerator())
{
    $key = $kvp.Key
    $value = $kvp.Value
    $path = Join-Path ${value} $key

    $exist = Test-Path("${key}")
    if ($exist)
    {
        Remove-Item $key | Out-Null
    }

    $exist = Test-Path("${path}")
    if (!$exist)
    {
        continue
    }

    New-Item -ItemType SymbolicLink -Path "${key}" -Value "${path}" | Out-Null
}
Pop-Location