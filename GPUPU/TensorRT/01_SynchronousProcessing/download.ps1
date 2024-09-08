$modelFile = "ResNet101-DUC-12.onnx"
$url = "https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/duc/model/ResNet101-DUC-12.onnx?download="
$sha1 = "57F6A1FF384BDD8B8AE7F0F327FC859C2592E941"

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
        Write-Host "Model file is already downloaded but SHA1 is not matched (${hash})" -ForegroundColor Yellow
    }
}

if (!$exist)
{
    Write-Host "Download ${modelFile} from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${modelFile}"
}