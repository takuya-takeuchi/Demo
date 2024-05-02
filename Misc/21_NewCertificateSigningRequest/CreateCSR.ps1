#***************************************
#Arguments
#%1: Common Name. You can specify FQDN (e.g. www,contoso.com) or your name (e.g. Taro Yamada) if use ceritificate for client auth
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $commonName
)

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

$privateKey = "server.pem"
$certificateSigningRequest = "server.csr"

$C="JP"                # Country Name
$ST="Tokyo"            # State or Province Name
$L="Minato-ku"         # Locality Name
$O="Contoso"           # Organization Name
$OU="Docs,Contoso"     # Organizational Unit Name
$CN="${commonName}"    # Common Name

Write-Host "Create private key ..." -ForegroundColor Blue
& "${openssl}" genpkey -algorithm RSA -out "${privateKey}"
Write-Host "Create certificate signing request ..." -ForegroundColor Blue
& "${openssl}" req -new `
                   -key "${privateKey}" `
                   -out "${certificateSigningRequest}" `
                   -config "${opensslConfig}" `
                   -subj "/C=${C}/ST=${ST}/L=${L}/CN=${CN}/OU=${OU}/O=${O}" `
                   -addext "subjectAltName = DNS:${commonName}"