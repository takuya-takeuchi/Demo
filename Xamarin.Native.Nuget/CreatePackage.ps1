#***************************************
#Arguments
#%1: Url of nuget server (https://localserver:5000)
#%2: Key of nuget server's APIKEY 
#***************************************
Param([Parameter(
      Mandatory=$True,
      Position = 1
      )][string]
      $Url,

      [Parameter(
      Mandatory=$True,
      Position = 2
      )][string]
      $Key
)

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
   Invoke-Expression "dotnet nuget delete Native.Xamarin 1.0.0 -s ${Url}/v3/index.json -k ${Key} --non-interactive "
   Invoke-Expression "dotnet nuget push Native.Xamarin.1.0.0.nupkg -s ${Url}/v3/index.json -k ${Key} --skip-duplicate"
}
else
{
   Invoke-Expression "mono ${nugetPath} pack ${nuspec}"
   Invoke-Expression "dotnet nuget delete Native.Xamarin 1.0.0 -s ${Url}/v3/index.json -k ${Key} --non-interactive "
   Invoke-Expression "dotnet nuget push Native.Xamarin.1.0.0.nupkg -s ${Url}/v3/index.json -k ${Key} --skip-duplicate"
}

if ($lastexitcode -ne 0)
{
   Write-Host "Failed '${nugetPath} pack ${nuspec}" -ForegroundColor Red
   exit -1
}