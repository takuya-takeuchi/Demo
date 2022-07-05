$Targets = @(
    "DemoCSharp",
    "DemoVisualBasic"
)

foreach ($Target in $Targets)
{
    $Path = Join-Path sources ${Target}
    Push-Location $Path
        Write-Host "Run ${Target}"
        dotnet run -c Release
    Pop-Location
}