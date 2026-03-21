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
$copnfigPath = Join-Path $current "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json

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

Push-Location $target

git fetch -ap
git checkout $version
git submodule update --init --recursive .

$Configuration = $Configuration.ToLower()

if ($global:IsWindows)
{
    meson subprojects update
    meson setup $buildDir --vsenv --buildtype=$Configuration --prefix=$installDir
    meson configure $buildDir -Drtsp_server=enabled
    meson compile -C $buildDir
    meson install -C $buildDir
}
elseif ($global:IsMacOS)
{
    # sudo apt install libgtk-3-dev libsdl2-dev
    meson subprojects update
    meson setup $buildDir --buildtype=$Configuration --prefix=$installDir
    meson configure $buildDir -Drtsp_server=enabled
    meson compile -C $buildDir
    meson install -C $buildDir
}
elseif ($global:IsLinux)
{
    meson subprojects update
    meson setup $buildDir --buildtype=$Configuration --prefix=$installDir
    meson configure $buildDir -Drtsp_server=enabled
    meson compile -C $buildDir
    meson install -C $buildDir
}
Pop-Location