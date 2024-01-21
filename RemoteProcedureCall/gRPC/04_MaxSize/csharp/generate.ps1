$current = $PSScriptRoot

function get-protoc
{
    $path = (dotnet nuget locals global-packages --list).Replace('info : global-packages: ', '').Trim()
    if ($path)
    {
        $path = (dotnet nuget locals global-packages --list).Replace('global-packages: ', '').Trim()
    }

    $path = Join-Path $path "grpc.tools"
    if (!(Test-Path($path)))
    {
        Write-Host "[Error] grpc.tools nuget package is not installed" -ForegroundColor Red -InformationAction Continue
        return
    }

    $path = Get-ChildItem -Directory $path | Sort-Object Name -Descending | Select-Object -First 1
    if (!($path))
    {
        Write-Host "[Error] no versions of grpc.tools are installed" -ForegroundColor Red -InformationAction Continue
        return
    }

    if ($global:IsWindows)
    {
        $os = "windows_x64"
        $protoc = "protoc.exe"
    }
    elseif ($global:IsMacOS)
    {
        $os = "macosx_x64"
        $protoc = "protoc"
    }
    else
    {
        $os = "linux_x64"
        $protoc = "protoc"
    }

    $protoc = Join-Path $path tools | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $protoc
    if (!(Test-Path($protoc)))
    {
        Write-Host "[Error] protoc is missing in '${protoc}'" -ForegroundColor Red -InformationAction Continue
        return
    }

    Write-Host "[Info] protoc: ${protoc}" -ForegroundColor Green -InformationAction Continue

    $version = & "${protoc}" --version
    Write-Host "[Info] ${version}" -ForegroundColor Green -InformationAction Continue

    return $protoc
}

function get-grpc_csharp_plugin
{
    param ($protoc)

    if (!(Test-Path($protoc)))
    {
        Write-Host "[Error] protoc is missing in '${protoc}'" -ForegroundColor Red -InformationAction Continue
        return
    }

    if ($global:IsWindows)
    {
        $grpc_csharp_plugin = "grpc_csharp_plugin.exe"
    }
    elseif ($global:IsMacOS)
    {
        $grpc_csharp_plugin = "grpc_csharp_plugin"
    }
    else
    {
        $grpc_csharp_plugin = "grpc_csharp_plugin"
    }

    $directory = Split-Path -Parent $protoc
    $grpc_csharp_plugin = Join-Path $directory $grpc_csharp_plugin
    if (!(Test-Path($grpc_csharp_plugin)))
    {
        Write-Host "[Error] grpc_csharp_plugin is missing in '${grpc_csharp_plugin}'" -ForegroundColor Red -InformationAction Continue
        return
    }

    Write-Host "[Info] grpc_csharp_plugin: ${grpc_csharp_plugin}" -ForegroundColor Green -InformationAction Continue

    return $grpc_csharp_plugin
}

$protoc = get-protoc
if (!($protoc))
{
    return
}

$grpc_csharp_plugin = get-grpc_csharp_plugin $protoc
if (!($grpc_csharp_plugin))
{
    return
}

$protofile = "example.proto"
$proto_path = Split-Path -Parent $current | `
              Join-Path -ChildPath protos
$proto = Join-Path $proto_path $protofile
if (!(Test-Path($proto)))
{
    Write-Host "[Error] proto is missing in '${proto}'" -ForegroundColor Red
    return
}

Write-Host "[Info] proto_path: ${proto_path}" -ForegroundColor Green
Write-Host "[Info] protofile: ${protofile}" -ForegroundColor Green

$outputs = @(
    (Join-Path "sources" "Client"),
    (Join-Path "sources" "Server")
)
foreach ($output in $outputs)
{
    & "${protoc}"--csharp_out "${output}" --grpc_out "${output}" --plugin=protoc-gen-grpc="${grpc_csharp_plugin}" --proto_path="${proto_path}" $protofile
}
# & "${protoc}"--csharp_out Client --grpc_out Client --plugin=protoc-gen-grpc="${grpc_csharp_plugin}" --proto_path="${proto_path}" $protofile
# & "${protoc}"--csharp_out Server --grpc_out Server --plugin=protoc-gen-grpc="${grpc_csharp_plugin}" --proto_path="${proto_path}" $protofile