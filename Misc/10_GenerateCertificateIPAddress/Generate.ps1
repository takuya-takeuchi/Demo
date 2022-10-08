Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $IpAddress
)

$C="JP"              # Country Name
$ST="Tokyo"          # State or Province Name
$L="Minato-ku"       # Locality Name
$O="Contoso"         # Organization Name
$OU="Docs,Contoso"   # Organizational Unit Name

$version = "1.0.2u"

if ($global:IsWindows)
{
   $openssl = Join-Path $PSScriptRoot openssl | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath windows | `
              Join-Path -ChildPath openssl.exe
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsMacOS)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsLinux)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}

& "${openssl}" req -newkey rsa:2048 `
                   -days 3650 `
                   -nodes `
                   -x509 `
                   -subj "/C=${C}/ST=${ST}/L=${L}/O=${O}/OU=${OU}/CN=${IpAddress}" `
                   -extensions v3_req `
                   -config "${opensslConfig}" `
                   -keyout server.key `
                   -out server.crt

if (Test-Path(".rnd"))
{
   Remove-Item ".rnd"
}

& "${openssl}" x509 -text -noout -in server.crt