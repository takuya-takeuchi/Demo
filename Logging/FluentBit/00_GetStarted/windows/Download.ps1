
$target = "fluentbit"
$version = "3.0.2"
$filename = "fluent-bit-${version}-win64.zip"
$url = "https://packages.fluentbit.io/windows/${filename}"

$current = $PSScriptRoot

$packageDir = Join-Path $current $target | `
              Join-Path -ChildPath $version

if (!(Test-Path("${packageDir}")))
{
    Write-Host "Download ${target} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${filename}"

    New-Item -Type Directory $packageDir -Force | Out-Null
    Expand-Archive -Path "${filename}" -DestinationPath $current
    $basename = (Get-Item $filename).BaseName
    Move-Item "$basename/*" $packageDir -Force | Out-Null

    Remove-Item "${filename}" -Recurse -Force | Out-Null
}

if (Test-Path("${filename}"))
{
    Remove-Item "${filename}" -Force | Out-Null
}

if (Test-Path("${basename}"))
{
    Remove-Item "${basename}" -Recurse -Force | Out-Null
}