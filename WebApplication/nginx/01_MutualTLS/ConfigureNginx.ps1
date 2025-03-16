$current = $PSScriptRoot

if ($global:IsWindows)
{
    $file = "nginx-1.26.1.zip"
    $expandedDir = Join-Path $current "nginx-1.26.1"
    $url = "https://nginx.org/download/${file}"
    $sha1 = "AF2E1D90BCD7754ACBBDDEFB88159557C1C2BEC3"
}
elseif ($global:IsMacOS)
{
    $file = "nginx-1.26.1.tar.gz"
    $url = "https://nginx.org/download/${file}"
    $sha1 = "A73998570100134004D665E81783B2A2FF808BCD"
}
elseif ($global:IsLinux)
{
    $file = "nginx-1.26.1.tar.gz"
    $url = "https://nginx.org/download/${file}"
    $sha1 = "A73998570100134004D665E81783B2A2FF808BCD"
}

# check certificates
$files = @(
   "ca.crt"
   "ca.key"
   "server.crt"
   "server.key"
   "client.crt"
   "client.decrypted.key"
)

foreach ($file in $files)
{
   if (!(Test-Path($file)))
   {
      Write-Host "$file is not found" -ForegroundColor Red
      exit
   }
}

# copy nginx.conf
Copy-Item ("${current}/conf/nginx.conf") ("nginx/conf") -Force