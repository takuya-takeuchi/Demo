$root = $PSScriptRoot
$kafka_root = Join-Path $root kafka
if (!(Test-Path($kafka_root)))
{
    Write-Host "${kafka_root} is missing" -ForegroundColor Red
    exit
}

Push-Location $kafka_root

$kafka_config = Join-Path config server.properties
$zookeeper_config = Join-Path config zookeeper.properties
                    
if (!(Test-Path($kafka_config)))
{
    Write-Host "${kafka_config} is missing" -ForegroundColor Red
    exit
}
if (!(Test-Path($zookeeper_config)))
{
    Write-Host "${zookeeper_config} is missing" -ForegroundColor Red
    exit
}

if ($IsWindows)
{
    $kafka_start_shell = Join-Path $kafka_root bin | `
                         Join-Path -ChildPath windows | `
                         Join-Path -ChildPath kafka-server-start.bat
    $zookeeper_start_shell = Join-Path $kafka_root bin | `
                             Join-Path -ChildPath windows | `
                             Join-Path -ChildPath zookeeper-server-start.bat
}
else
{
    $kafka_start_shell = Join-Path bin kafka-server-start.sh
    $zookeeper_start_shell = Join-Path bin zookeeper-server-start.sh
}
                    
if (!(Test-Path($kafka_start_shell)))
{
    Write-Host "${kafka_start_shell} is missing" -ForegroundColor Red
    exit
}
if (!(Test-Path($zookeeper_start_shell)))
{
    Write-Host "${zookeeper_start_shell} is missing" -ForegroundColor Red
    exit
}

if (!(Test-Path($env:JAVA_HOME)))
{
    Write-Host "JAVA_HOME is missing" -ForegroundColor Red
    exit
}

Write-Host "    Kafka Shell: ${kafka_start_shell}" -ForegroundColor Blue
Write-Host "ZooKeeper Shell: ${zookeeper_start_shell}" -ForegroundColor Blue
Write-Host "    Kafka config: ${kafka_config}" -ForegroundColor Blue
Write-Host "ZooKeeper config: ${zookeeper_config}" -ForegroundColor Blue

$zookeeper = Start-Process "${zookeeper_start_shell}" -ArgumentList "${zookeeper_config}" -PassThru
$kafka = Start-Process "${kafka_start_shell}" -ArgumentList "${kafka_config}" -PassThru

Wait-Process -Id ($zookeeper.id)
Wait-Process -Id ($kafka.id)

Pop-Location