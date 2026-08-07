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

$sourceDir = $current
$buildDir = Join-PathArray -PathElements @($current, "build", $os,  "program", $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os)
$installBinaryDir = Join-PathArray -PathElements @($installDir, "bin")

$target = "spdlog"
$version = $config.${target}.version
$shared = $config.$target.shared ? "dynamic" : "static"

$installDirs = @{}
$installDirs[$target]        = Join-PathArray -PathElements @($rootDir, "install", $os, $target, $config.${target}.version, ($config.$target.shared ? "dynamic" : "static"), $Configuration)
$installDirs["spdlog_setup"] = Join-PathArray -PathElements @($rootDir, "install", $os, "spdlog_setup", $config.spdlog_setup.version, ($config.spdlog_setup.shared ? "dynamic" : "static"), $Configuration)

foreach ($key in $installDirs.Keys)
{
    $dir = $installDirs[$key]
    if (!(Test-Path(${dir})))
    {
        Write-Host "[Error] ${dir} is missing" -ForegroundColor Red
        return
    }
}

# Do use ';' as separator for CMAKE_PREFIX_PATH on all platforms, because CMake will convert it to ':' on Linux and macOS automatically.
$cmakePrefixPath = $installDirs.Values -join ";"

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

# build
Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vsVersion = $config.windows.visualStudioVersion
        $vs = "C:\Program Files\Microsoft Visual Studio\${vsVersion}"
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
    $vsVersion = $config.windows.visualStudioVersion
    $vsInternalVersion = $config.windows.visualStudioInternalVersion

    $cmakeArgs += @(
        "-G", "Visual Studio ${vsInternalVersion} ${vsVersion}", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${cmakePrefixPath}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${cmakePrefixPath}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${cmakePrefixPath}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
    )
}

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location