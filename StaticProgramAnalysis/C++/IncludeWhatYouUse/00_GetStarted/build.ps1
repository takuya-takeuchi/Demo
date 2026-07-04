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

function Join-PathArray {
    [CmdletBinding()]
    param (
        [Parameter(Mandatory = $true, ValueFromPipeline = $true)]
        [string[]]$PathElements
    )

    process {
        if ($PathElements.Count -eq 0) { return }
        $result = $PathElements[0]
        for ($i = 1; $i -lt $PathElements.Count; $i++) { $result = Join-Path -Path $result -ChildPath $PathElements[$i] }
        return $result
    }
}


$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$configPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json

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

$target = "include-what-you-use"
$version = $config."${target}".version
$version = $config."${target}".mapping."${version}"
if ($config."${target}".shared)
{
    $shared = "dynamic"
}
else
{
    $shared = "static"
}

# build
$sourceDir = $current
$buildDir = Join-PathArray -PathElements @($current, "build", $os, "program", $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os)

$targetInstallDir = Join-PathArray -PathElements @($rootDir, "install", $os, $target, $version, $shared, $Configuration)
if (!(Test-Path(${targetInstallDir})))
{
    Write-Host "[Error] ${targetInstallDir} is missing" -ForegroundColor Red
    return
}
$IWYU_TOOL = Get-ChildItem -Path "${targetInstallDir}" -Filter "iwyu_tool.py" -Recurse -File
if (!(Test-Path(${IWYU_TOOL})))
{
    Write-Host "[Error] ${IWYU_TOOL} is missing" -ForegroundColor Red
    return
}

$llvmVersion = $config.llvm.version
$LLVM_INSTALL_DIR = Join-PathArray -PathElements @($rootDir, "install", $os, "llvm", $llvmVersion)
$CMAKE_C_COMPILER = Get-ChildItem -Path "${LLVM_INSTALL_DIR}" -Filter "clang" -Recurse -File
$CMAKE_CXX_COMPILER = Get-ChildItem -Path "${LLVM_INSTALL_DIR}" -Filter "clang++" -Recurse -File

$paths = @(
    "${CMAKE_C_COMPILER}"
    "${CMAKE_CXX_COMPILER}"
)
foreach ($path in $paths)
{
    if (!(Test-Path(${path})))
    {
        Write-Host "[Error] ${path} is missing" -ForegroundColor Red
        return
    }
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

$cmakeArgs = @()
if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vs = "C:\Program Files\Microsoft Visual Studio\2022"
        $path = "${vs}\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Professional\VC\Auxiliary\Build\vcvars64.bat"
        }
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Community\VC\Auxiliary\Build\vcvars64.bat"
        }

        Write-Host "Use: ${path}" -ForegroundColor Green

        cmd.exe /c "call `"${path}`" && set > %temp%\vcvars.txt"
        Get-Content "${env:temp}\vcvars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    CallVisualStudioDeveloperConsole
    chcp 65001

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${targetInstallDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D PostgreSQL_ROOT=${libpqInstallDir}"
        "-D libpqxx_LIBRARY_DIR=${targetInstallDir}/bin"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${targetInstallDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D PostgreSQL_ROOT=${libpqInstallDir}"
        "-D libpqxx_LIBRARY_DIR=${targetInstallDir}/lib"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_C_COMPILER=${CMAKE_C_COMPILER}",
        "-D CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    )
}

$cmakeArgs += @(
    "-D CMAKE_EXPORT_COMPILE_COMMANDS=ON"
)

$cmakeArgs += @(
    "-B ${buildDir}"
    "${sourceDir}"
)

$configLogFile = Join-PathArray -PathElements @($buildDir, "cmake-config.log")
$buildLogFile = Join-PathArray -PathElements @($buildDir, "cmake-build.log")

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build "${buildDir}" --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

python3 "${IWYU_TOOL}" -p "${buildDir}"