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

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir

git fetch -ap
git checkout $version
git submodule update --init --recursive .

$Configuration = $Configuration.ToLower()

$setupArgs = @()
if ($global:IsWindows)
{
    # IMPORTANT!!!!
    # https://github.com/mesonbuild/meson/issues/12979
    chcp 65001

    $setupArgs += @(
        "--vsenv"
    )
}
elseif ($global:IsMacOS)
{
    # sudo apt install libgtk-3-dev libsdl2-dev
}
elseif ($global:IsLinux)
{
}

$setupArgs += @(
    "--buildtype=$Configuration"
    "--prefix=$installDir"
    "-Dgpl=disabled"
    "-Dauto_features=disabled"

    "-Dpython=disabled"
    "-Dlibav=disabled"
    "-Dlibnice=disabled"
    "-Ddevtools=disabled"
    "-Dges=disabled"
    "-Drtsp_server=disabled"
    "-Dsharp=disabled"
    "-Drs=disabled"
    "-Dgst-examples=disabled"
    "-Dtls=disabled"
    "-Dqt5=disabled"

    "-Dgst-plugins-good:rtp=enabled"
    "-Dgst-plugins-good:rtpmanager=enabled"
    "-Dgst-plugins-good:rtsp=enabled"
    "-Dgst-plugins-good:jpeg=enabled"

    "-Dgst-plugins-base:app=enabled" # appsink
    "-Dgst-plugins-base:videoconvertscale=enabled" # videoconvert

    "-Dgst-plugins-good:soup=disabled"
    "-Dgst-plugins-bad:webrtc=disabled"
    "-Dgst-plugins-bad:webrtcdsp=disabled"
    "-Dgst-plugins-bad:srtp=disabled"
    "-Dgst-plugins-bad:sctp=disabled"
    "-Dlibnice=disabled"
    "-Dwebrtc=disabled"
    "-Dgst-plugins-bad:videoparsers=enabled"
    "-Dgst-plugins-bad:opencv=disabled"
    "-Dgst-plugins-bad:openh264=enabled"

    "-Dgst-plugins-bad:nvdswrapper=disabled"
    "-Dgst-plugins-bad:nvcomp=disabled"
    "-Dgst-plugins-bad:cuda-nvmm=disabled"
    "-Dgst-plugins-bad:cuda-nvmm-include-path=disabled"
    "-Dgst-plugins-bad:nvcodec=disabled"
    "-Dgst-plugins-bad:vulkan=disabled"
    "-Dgst-plugins-bad:vulkan-video=disabled"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

meson subprojects update
meson setup $buildDir @setupArgs 2>&1 | Tee-Object -FilePath $configLogFile
meson compile -C $buildDir 2>&1 | Tee-Object -FilePath $buildLogFile
meson install -C $buildDir

Pop-Location