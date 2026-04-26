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
else
{
    Write-Host "[Error] This platfor is not supported" -ForegroundColor Red
    exit
}

$target = "pkg-config"
$pkgConfigversion = $config.pkg_config.version
$glibVersion = $config.glib.version
$getTextRuntimeVersion = $config.gettext_runtime.version

$pkgConfigZipName = "pkg-config_${pkgConfigversion}_win64.zip"
$glibZipName= "glib_${glibVersion}_win64.zip"
$getTextRuntimeZipName = "gettext-runtime_${getTextRuntimeVersion}_win64.zip"

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target
New-Item -ItemType Directory -Path $installDir -Force | Out-Null

function Get-FileSha256 {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return $null
    }

    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Download-IfNeeded {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Url,

        [Parameter(Mandatory = $true)]
        [string]$OutFile,

        [Parameter(Mandatory = $true)]
        [string]$ExpectedSha256
    )

    $ExpectedSha256 = $ExpectedSha256.ToLowerInvariant()

    if (Test-Path -LiteralPath $OutFile) {
        $currentHash = Get-FileSha256 -Path $OutFile
        if ($currentHash -eq $ExpectedSha256) {
            Write-Host "[Info] Hash matched: $OutFile" -ForegroundColor Yellow
            return
        }

        Write-Host "[Info] Hash: $currentHash" -ForegroundColor Yellow
        Write-Host "[Info] Hash mismatch: $OutFile" -ForegroundColor Yellow
        Remove-Item -LiteralPath $OutFile -Force
    }

    Write-Host "[Info] $Url" -ForegroundColor Yellow
    Invoke-WebRequest -Uri $Url -OutFile $OutFile

    $downloadedHash = Get-FileSha256 -Path $OutFile
    if ($downloadedHash -ne $ExpectedSha256) {
        Remove-Item -LiteralPath $OutFile -Force -ErrorAction SilentlyContinue
        throw "SHA256 mismatch for '$OutFile'. Expected=$ExpectedSha256 Actual=$downloadedHash"
    }

    Write-Host "[Info] Verified: $OutFile" -ForegroundColor Yellow
}

function Expand-ZipForce {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ZipPath,

        [Parameter(Mandatory = $true)]
        [string]$Destination
    )

    if (-not (Test-Path -LiteralPath $Destination)) {
        New-Item -ItemType Directory -Path $Destination | Out-Null
    }

    Write-Host "[Info] Expand: $ZipPath -> $Destination" -ForegroundColor Yellow
    Expand-Archive -LiteralPath $ZipPath -DestinationPath $Destination -Force
}

$packages = @(
    @{
        Url    = "https://download.gnome.org/binaries/win64/glib/2.26/${glibZipName}"
        File   = ${glibZipName}
        Sha256 = $config.glib.sha256
    },
    @{
        Url    = "https://download.gnome.org/binaries/win64/dependencies/${getTextRuntimeZipName}"
        File   = ${getTextRuntimeZipName}
        Sha256 = $config.gettext_runtime.sha256
    },
    @{
        Url    = "https://download.gnome.org/binaries/win64/dependencies/${pkgConfigZipName}"
        File   = ${pkgConfigZipName}
        Sha256 = $config.pkg_config.sha256
    }
)

foreach ($pkg in $packages)
{
    $zipPath = Join-Path $installDir $pkg.File
    Download-IfNeeded -Url $pkg.Url -OutFile $zipPath -ExpectedSha256 $pkg.Sha256
    Expand-ZipForce -ZipPath $zipPath -Destination $installDir
}

Write-Host "[Info] All packages are ready in: $installDir" -ForegroundColor Yellow