$current = $PSScriptRoot

$clients = @(
    "Client",
    "LegacyClient"
)

$sourceRoot = Join-Path $current "sources"

$count = 1000
$size = 1000 * 1000 * 10

foreach ($client in $clients)
{
    $directoryName = $client
    $projectDir = Join-Path $sourceRoot $directoryName

    Push-Location "${projectDir}"

    Write-Host "Test ${projectDir}" -ForegroundColor Green
    dotnet run -c Release -- $count $size

    Pop-Location
}