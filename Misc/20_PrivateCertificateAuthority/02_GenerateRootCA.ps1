$version = "3.3.0"

$current = $PSScriptRoot
if ($global:IsWindows)
{
   $openssl = Join-Path $current openssl | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath windows | `
              Join-Path -ChildPath x64 | `
              Join-Path -ChildPath bin | `
              Join-Path -ChildPath openssl.exe
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
   $env:OPENSSL_CONF=$opensslConfig
}
elseif ($global:IsMacOS)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsLinux)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}

if (!(Test-Path("root-ca.conf")))
{
   Write-Host "'root-ca.conf' is missing" -ForegroundColor Red
   exit
}

$caRoot = Join-Path $current pki | Join-Path -ChildPath CA
$rootCAConf = Join-Path $current root-ca.conf
$rootCAPrivateKey = Join-Path $caRoot private | Join-Path -ChildPath root-ca.key
$rootCACsr = Join-Path $caRoot root-ca.csr
$rootCACrt = Join-Path $caRoot root-ca.crt

Write-Host "Create private key and certificate signing request..." -ForegroundColor Blue
& "${openssl}" req -new -config "${rootCAConf}" -out "${rootCACsr}" -keyout "${rootCAPrivateKey}"
Write-Host "Do Self-Sign certificate signing request..." -ForegroundColor Blue
& "${openssl}" ca  -batch -selfsign -config "${rootCAConf}" -in "${rootCACsr}" -out "${rootCACrt}" -extensions req_ext