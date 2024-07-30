$modelFile = "resnetv2_50_Opset18.onnx"
$url = "https://github.com/onnx/models/raw/main/Computer_Vision/resnetv2_50_Opset18_timm/resnetv2_50_Opset18.onnx?download="
$sha1 = "9bbdf72de1f6f5f9a7bf1575534ab4bd70658623"

$exist = Test-Path(${modelFile})
if ($exist)
{
    $hash = (Get-FileHash ${modelFile} -Algorithm SHA1).hash
    $exist = $sha1 -eq $hash
    if ($exist)
    {
        Write-Host "Model file is already downloaded" -ForegroundColor Green
    }
    else
    {
        Write-Host "Model file is already downloaded but SHA1 is not matched" -ForegroundColor Yellow
    }
}

if (!$exist)
{
    Write-Host "Download ${modelFile} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${modelFile}"
}