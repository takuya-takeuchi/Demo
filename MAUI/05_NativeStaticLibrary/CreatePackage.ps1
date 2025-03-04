function Create()
{
   Param([string]$target)

   $nuspec = "${target}.nuspec"
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
   }

   if ($lastexitcode -ne 0)
   {
      Write-Host "Failed '${nugetPath} pack ${nuspec}" -ForegroundColor Red
      exit -1
   }
}

Create "Native.Maui"