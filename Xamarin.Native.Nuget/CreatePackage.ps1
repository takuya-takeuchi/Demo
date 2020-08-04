$nuspec = "Native.Xamarin.nuspec"
if (!(Test-Path ${nuspec}))
{
   Write-Host "Error: ${nuspec} does not exist" -ForegroundColor Red
   exit -1
}

$nugetPath = Join-Path $PSScriptRoot nuget.exe
if (!(Test-Path ${nugetPath}))
{
   Write-Host "Error: ${nugetPath} does not exist" -ForegroundColor Red
   exit -1
}

Write-Host "${nuspec}" -ForegroundColor Green

if ($global:IsWindows)
{
   Invoke-Expression "${nugetPath} pack ${nuspec}"
}
else
{
   Invoke-Expression "mono ${nugetPath} pack ${nuspec}"
   Invoke-Expression "dotnet nuget delete Native.Xamarin 1.0.0 -s http://192.168.11.17:50505/v3/index.json -k demonbane --non-interactive "
   Invoke-Expression "dotnet nuget push Native.Xamarin.1.0.0.nupkg -s http://192.168.11.17:50505/v3/index.json -k demonbane --skip-duplicate"
}

if ($lastexitcode -ne 0)
{
   Write-Host "Failed '${nugetPath} pack ${nuspec}" -ForegroundColor Red
   exit -1
}