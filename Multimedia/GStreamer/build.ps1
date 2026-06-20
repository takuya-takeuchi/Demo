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
$rootDir = $PSScriptRoot
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
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "gstreamer"
$version = $config.gstreamer.version

# python venv activate
$venvDir = Join-Path $current ".venv"
Write-Host "[Info] Activate python venv"
if ($global:IsWindows)
{
    if (!(Test-Path("${venvDir}")))
    {
        Write-Host "[Info] Create python venv"
        python -m venv "${venvDir}"
    }

    & "${venvDir}\Scripts\Activate.ps1"
    Write-Host "[Info] Activated"
}
elseif ($global:IsMacOS)
{
    if (!(Test-Path("${venvDir}")))
    {
        Write-Host "[Info] Create python venv"
        python3 -m venv "${venvDir}"
    }

    & "${venvDir}/bin/Activate.ps1"
    Write-Host "[Info] Activated"
}
elseif ($global:IsLinux)
{
    if (!(Test-Path("${venvDir}")))
    {
        Write-Host "[Info] Create python venv"
        python3 -m venv "${venvDir}"
    }

    & "${venvDir}/bin/Activate.ps1"
    Write-Host "[Info] Activated"
}

Write-Host "[Info] Check python executable location"
python -c "import sys; print(f'Python executable: {sys.executable}')"

python -m pip install pip --upgrade
python -m pip install meson ninja

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $version | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $Configuration

$opencvVersion = $config.opencv.version
if ($config.opencv.shared)
{
    $opencvShared = "dynamic"
}
else
{
    $opencvShared = "static"
}
$OPENCV_INSTALL_DIR = Join-Path $rootDir install | `
                      Join-Path -ChildPath $os | `
                      Join-Path -ChildPath opencv | `
                      Join-Path -ChildPath $opencvVersion | `
                      Join-Path -ChildPath $opencvShared | `
                      Join-Path -ChildPath $Configuration
$OPENCV_PKGCONFIG_DIR = Join-Path $OPENCV_INSTALL_DIR lib | `
                        Join-Path -ChildPath pkgconfig

$paths = @(
    "${OPENCV_PKGCONFIG_DIR}"
)
foreach ($path in $paths)
{
    if (!(Test-Path($path)))
    {
        Write-Host "${path} is missing" -ForegroundColor Red
        exit
    }        
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $target

git fetch -ap
git checkout $version
git submodule update --init --recursive .

$Configuration = $Configuration.ToLower()

$setupArgs = @()
if ($global:IsWindows)
{
    $setupArgs += @(
        "--vsenv"
        "--buildtype=$Configuration"
        "--prefix=$installDir"
        "-Dgpl=disabled"
    )
}
elseif ($global:IsMacOS)
{
    # sudo apt install libgtk-3-dev libsdl2-dev
    $setupArgs += @(
        "--buildtype=$Configuration"
        "--prefix=$installDir"
        "-Dgpl=disabled"
    )
}
elseif ($global:IsLinux)
{
    $setupArgs += @(
        "--buildtype=$Configuration"
        "--prefix=$installDir"
        "-Dgpl=disabled"

        "-Dgst-plugins-bad:opencv=disabled"
        "-Dgst-plugins-bad:openh264=enabled"
    )
    # $setupArgs += @(
    #     "--buildtype=$Configuration"
    #     "--prefix=$installDir"
    #     "-Dgpl=disabled"
    #     "-Dauto_features=disabled"
        
    #     # "-Dgst-plugins-good=enabled"
    #     # # "-Dgst-plugins-base:appsink=enabled"
    #     # # "-Dgst-plugins-base:videoconvert=enabled"
    #     # "-Dgst-plugins-good:rtspsrc=enabled"
    #     # "-Dgst-plugins-good:rtph264depay=enabled"
    #     # "-Dgst-plugins-good:jpegenc=enabled"
    #     # "-Dgst-plugins-bad:h264parse=enabled"
    #     # "-Dgst-plugins-bad:videoconvert=enabled"

    #     "-Dpython=disabled"
    #     "-Dlibav=disabled"
    #     "-Dlibnice=disabled"
    #     "-Ddevtools=disabled"
    #     "-Dges=disabled"
    #     "-Drtsp_server=disabled"
    #     "-Dvaapi=disabled"
    #     "-Dsharp=disabled"
    #     "-Drs=disabled"
    #     "-Dgst-examples=disabled"
    #     "-Dtls=disabled"
    #     "-Dqt5=disabled"

    #     "-Dbase=enabled"
    #     "-Dgood=enabled"
    #     "-Dbad=enabled"
    #     "-Dgst-plugins-bad:opencv=disabled"
    #     "-Dgst-plugins-bad:openh264=enabled"
    # )
}
    # "-Dgst-plugins-base:appsink=enabled"
    # "-Dgst-plugins-base:videoconvert=enabled"
    # "-Dgst-plugins-good:rtspsrc=enabled"
    # "-Dgst-plugins-good:rtph264depay=enabled"
    # "-Dgst-plugins-good:jpegenc=enabled"
    # "-Dgst-plugins-bad:h264parse=enabled"
    # "-Dgst-plugins-bad:openh264=enabled"
    # "-Dgst-plugins-bad:videoconvert=enabled"

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

# $env:PKG_CONFIG_PATH="${OPENCV_PKGCONFIG_DIR}:$PKG_CONFIG_PATH"

meson subprojects update
meson setup @setupArgs $buildDir 2>&1 | Tee-Object -FilePath $configLogFile
meson configure $buildDir -Drtsp_server=enabled
meson compile -C $buildDir 2>&1 | Tee-Object -FilePath $buildLogFile
meson install -C $buildDir

Pop-Location