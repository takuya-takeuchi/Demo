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

$projectRoot = Join-Path ${current} "sources" | `
               Join-Path -ChildPath "Demo"
$protoPath = Join-Path ${projectRoot} "Protos" 
$testRoot = Join-Path ${current} "tests" | `
            Join-Path -ChildPath "Demo.Test"
$outputDir = Join-Path ${testRoot} "Grpc"

New-Item -Type Directory -Force "${outputDir}" | Out-Null

& "${protoc}" --proto_path="${projectRoot}" `
              --proto_path="${protoPath}" `
              --csharp_out="${outputDir}" `
              --grpc_out="${outputDir}" `
              --plugin=protoc-gen-grpc="${plugin}" `
              option.proto
& "${protoc}" --proto_path="${projectRoot}" `
              --proto_path="${protoPath}" `
              --csharp_out="${outputDir}" `
              --grpc_out="${outputDir}" `
              --plugin=protoc-gen-grpc="${plugin}" `
              greet.proto