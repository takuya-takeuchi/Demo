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

function Copy-FilesAndLinksFlat {
    param (
        [Parameter(Mandatory=$true)]
        [string]$SourceDir,
        [Parameter(Mandatory=$true)]
        [string]$DestDir,
        [Parameter(Mandatory=$false)]
        [string]$ExtensionPattern = ".*"
    )
    if (!(Test-Path $DestDir)) { New-Item -ItemType Directory -Path $DestDir | Out-Null }
    Get-ChildItem -Path $SourceDir -Recurse -File | ForEach-Object {
        if ($_.Name -match $ExtensionPattern)
        {
            $targetPath = Join-Path $DestDir $_.Name
            if ($_.Attributes -match "ReparsePoint") {
                $linkTarget = (Get-Item $_.FullName).Target
                New-Item -ItemType SymbolicLink -Path $targetPath -Value $linkTarget -Force | Out-Null
            }
            else
            {
                Copy-Item $_.FullName -Destination $targetPath -Force
            }
        }
    }
}

$current = $PSScriptRoot
$configPath = Join-Path $current "build-config.json"
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

    # Check windows support for long paths
    $registryPath  = "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem"
    $valueName     = "LongPathsEnabled"
    $expectedValue = 1

    try
    {
        if (-not (Test-Path $registryPath)) {
            throw "Registry Path Not Found: $registryPath"
        }

        $regKey = Get-Item -Path $registryPath
        if (-not ($regKey.GetValueNames() -contains $valueName)) {
            throw "Value '$valueName' does not exist in $registryPath"
        }

        $currentValue = Get-ItemPropertyValue -Path $registryPath -Name $valueName
        if ($currentValue -ne $expectedValue) {
            throw "Value mismatch: '$valueName' is $currentValue (Expected: $expectedValue)"
        }
    }
    catch
    {
        Write-Host "[ERROR] $($_.Exception.Message)" -ForegroundColor Red
        exit 1
    }

    # Check git config for long paths
    if ((git config --global core.longpaths) -ne "true")
    {
        Write-Host "[ERROR] core.longpaths is not true!" -ForegroundColor Red
        exit 1
    }
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "amazon-kinesis-video-streams-producer-sdk-cpp"
$version = $config."${target}".version
# Fucking Visual Studio cannot handle long paths, so we use a shorter name for the build and install directories!!!!
# https://developercommunity.visualstudio.com/t/Allow-building-running-and-debugging-a/351628
$target = "kvs-sdk-cpp"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-PathArray -PathElements @($current, "build", $os, $target, $version, $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os, $target, $version, $Configuration)

$gstreamerInstallDir = Join-PathArray -PathElements @($current, "install", $os, "gstreamer-kvs", $config.gstreamer.version, $Configuration)
$GSTREAMER_PKGCONFIG_DIR = (Get-ChildItem -Path $gstreamerInstallDir -Recurse -Directory | Where-Object { $_.Name -eq "pkgconfig" } | Select-Object -First 1).FullName
if (!(Test-Path(${GSTREAMER_PKGCONFIG_DIR})))
{
    Write-Host "[Error] ${GSTREAMER_PKGCONFIG_DIR} is missing" -ForegroundColor Red
    return
}
$gstreamerInstallDir = $gstreamerInstallDir.Replace("`\", "/")

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $current
git submodule update --init --recursive .
Pop-Location

Push-Location $sourceDir
git fetch --all --prune
git checkout $version
git submodule update --init --recursive .
Pop-Location

# apply patch
$patch = Join-Path $current patch |
         Join-Path -ChildPath $target |
         Join-Path -ChildPath $version |
         Join-Path -ChildPath $os
if (Test-Path($patch))
{
    Copy-Item -Recurse $patch/* $sourceDir -Force
}

Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsWindows)
{
    $perlDir = $env:PERLPATH
    if (!$perlDir)
    {
        Write-Host "[Error] Environmental Variable: PERLPATH is missing" -ForegroundColor Red
        exit
    }

    if (!(Test-Path("${perlDir}")))
    {
        Write-Host "[Error] '${perlDir}' is missing" -ForegroundColor Red
        exit
    }

    $env:PATH="${perlDir};${env:PATH}"
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

    $pkgConfigExe = Join-PathArray -PathElements @($current, "install", $os, "pkg-config", "bin", "pkg-config.exe")
    if (!(Test-Path(${pkgConfigExe})))
    {
        Write-Host "[Error] ${pkgConfigExe} is missing. Please run download-pkg-config.ps1" -ForegroundColor Red
        return
    }

    $vsVersion = $config.windows.visualStudioVersion
    $vsInternalVersion = $config.windows.visualStudioInternalVersion

    $cmakeArgs += @(
        "-G", "Visual Studio ${vsInternalVersion} ${vsVersion}", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${gstreamerInstallDir}"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D PKG_CONFIG_EXECUTABLE=${pkgConfigExe}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${gstreamerInstallDir}"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${gstreamerInstallDir}"
        "-D CMAKE_C_FLAGS=-D_GNU_SOURCE"
        "-D CMAKE_CXX_FLAGS=-D_GNU_SOURCE"
    )
}

# standard
$cmakeArgs += @(
    "-D BUILD_GSTREAMER_PLUGIN=ON"
    "-D BUILD_DEPENDENCIES=ON"
    "-D BUILD_TEST=OFF"
)

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

$env:PKG_CONFIG_PATH = "${GSTREAMER_PKGCONFIG_DIR}"
cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

# https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp/issues/978
if ($global:IsMacOS)
{
    cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
    cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
}

# copy dependency files
$source = Join-PathArray -PathElements @($sourceDir, "open-source", "local", "lib")
if ($global:IsWindows)
{
}
elseif ($global:IsMacOS)
{
    Copy-FilesAndLinksFlat "${buildDir}" "${installDir}/lib" ".*\.a(\.\d+)*$"
    Copy-FilesAndLinksFlat "${buildDir}" "${installDir}/lib" ".*\.dylib(\.\d+)*$"
    Copy-FilesAndLinksFlat "${buildDir}" "${installDir}/lib" ".*\.so(\.\d+)*$"
    Copy-FilesAndLinksFlat "${source}" "${installDir}/lib" ".*\.dylib(\.\d+)*$"
}
elseif ($global:IsLinux)
{
    Copy-FilesAndLinksFlat "${buildDir}" "${installDir}/lib" ".*\.a(\.\d+)*$"
    Copy-FilesAndLinksFlat "${buildDir}" "${installDir}/lib" ".*\.so(\.\d+)*$"
    Copy-FilesAndLinksFlat "${source}" "${installDir}/lib" ".*\.so(\.\d+)*$"
}

Pop-Location