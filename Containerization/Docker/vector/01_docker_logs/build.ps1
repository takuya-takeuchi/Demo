Param
(
)

$dockerImageName = "demo-vector-sidecar-app"

$current = $PSScriptRoot
$appDir = Join-Path $current app

Push-Location $appDir
docker build -t $dockerImageName -f Dockerfile .
Pop-Location