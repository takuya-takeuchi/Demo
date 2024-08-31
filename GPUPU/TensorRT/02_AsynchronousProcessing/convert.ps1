$modelFile = "ResNet101-DUC-12.onnx"
$outputFile = "ResNet101-DUC-12.trt"

$exist = Test-Path(${modelFile})
if (!$exist)
{
    Write-Host "${modelFile} is missing" -ForegroundColor Red
    exit
}

$cudaPath = $env:CUDA_PATH
if (!$cudaPath)
{
    Write-Host "Environmental Variable 'CUDA_PATH' is missing" -ForegroundColor Red
    exit
}

$trtPath = $env:TENSORRT_PATH
if (!$trtPath)
{
    Write-Host "Environmental Variable 'TENSORRT_PATH' is missing" -ForegroundColor Red
    exit
}

$cudaBinDir = Join-Path "${cudaPath}" bin
$tensorRTBinDir = Join-Path "${trtPath}" bin
$tensorRTLibDir = Join-Path "${trtPath}" lib

$binaries = @{
    "trtexec.exe"                     = "${tensorRTBinDir}";
    # TensorRT 10
    "nvinfer_10.dll"                  = "${tensorRTLibDir}";
    "nvinfer_plugin_10.dll"           = "${tensorRTLibDir}";
    "nvonnxparser_10.dll"             = "${tensorRTLibDir}";
    "nvinfer_builder_resource_10.dll" = "${tensorRTLibDir}";
    # TensorRT 8
    "nvinfer.dll"                     = "${tensorRTLibDir}";
    "nvinfer_plugin.dll"              = "${tensorRTLibDir}";
    "nvonnxparser.dll"                = "${tensorRTLibDir}";
    "nvparsers.dll"                   = "${tensorRTLibDir}";
    "cudart64_110.dll"                = "${cudaBinDir}"
    "cublas64_11.dll"                 = "${cudaBinDir}"
    "cublasLt64_11.dll"               = "${cudaBinDir}"
}

foreach ($kvp in $binaries.GetEnumerator())
{
    $key = $kvp.Key
    $value = $kvp.Value
    $path = Join-Path ${value} $key

    $exist = Test-Path("${key}")
    if ($exist)
    {
        Remove-Item $key | Out-Null
    }

    $exist = Test-Path("${path}")
    if (!$exist)
    {
        continue
    }

    New-Item -ItemType SymbolicLink -Path "${key}" -Value "${path}" | Out-Null
}

# not use `--explicitBatch` because I need using dynamic shape
$trtexec = Join-Path $PSScriptRoot "trtexec.exe"
& "${trtexec}" --onnx="${modelFile}" --saveEngine="${outputFile}" 2>&1 | Tee-Object -FilePath trtexec.log | ForEach-Object { $_ }

foreach ($kvp in $binaries.GetEnumerator())
{
    $key = $kvp.Key
    $value = $kvp.Value
    $path = Join-Path ${value} $key

    $exist = Test-Path("${key}")
    if ($exist)
    {
        Remove-Item $key | Out-Null
    }
}