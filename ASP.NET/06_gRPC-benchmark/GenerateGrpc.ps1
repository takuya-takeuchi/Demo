$current = $PSScriptRoot
$nugetPackageRoot = (dotnet nuget locals --list global-packages --force-english-output).Replace("global-packages: ", "")

if (!(Test-Path("${nugetPackageRoot}")))
{
    Write-Host "global-packages: ${nugetPackageRoot} is missing" -ForegroundColor Red
    exit -1
}

$grpcVersion = "2.47.0"
$architecture = "x64"
if ($global:IsWindows)
{
    $os = "windows"
    $protoc = "protoc.exe"
    $plugin = "grpc_csharp_plugin.exe"
}
elseif ($global:IsMacOS)
{
    $os = "macosx"
    $protoc = "protoc"
    $plugin = "grpc_csharp_plugin"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $protoc = "protoc"
    $plugin = "grpc_csharp_plugin"
}
else
{
    Write-Host "Error: This plaform is not support" -ForegroundColor Red
    exit -1
}

$path = Join-Path ${nugetPackageRoot} "grpc.tools" | `
        Join-Path -ChildPath $grpcVersion | `
        Join-Path -ChildPath tools | `
        Join-Path -ChildPath "${os}_${architecture}"
$protoc = Join-Path ${path} ${protoc}
$plugin = Join-Path ${path} ${plugin}

if (!(Test-Path("${protoc}")))
{
    Write-Host "protoc: ${protoc} is missing" -ForegroundColor Red
    exit -1
}
if (!(Test-Path("${plugin}")))
{
    Write-Host "grpc_csharp_plugin: ${plugin} is missing" -ForegroundColor Red
    exit -1
}

$sourceRoot = Join-Path ${current} "sources"
$projectRoot = Join-Path ${sourceRoot} "Server"
$protoPath = Join-Path ${projectRoot} "Protos"

if (!(Test-Path("${protoPath}")))
{
    Write-Host "Protos: ${protoPath} is missing" -ForegroundColor Red
    #exit -1
}

$clients = @(
    "Client"
)

foreach ($client in $clients)
{
    $outputDir = Join-Path $sourceRoot $client
    $outputGrpcDir = Join-Path $outputDir "Grpc"
    New-Item -Type Directory -Force "${outputGrpcDir}" | Out-Null
    Write-Host "Output: ${outputGrpcDir}" -ForegroundColor Green
    
    & "${protoc}" --proto_path="${protoPath}" `
                  --csharp_out="${outputGrpcDir}" `
                  --grpc_out="${outputGrpcDir}" `
                  --plugin=protoc-gen-grpc="${plugin}" `
                  test.proto
}