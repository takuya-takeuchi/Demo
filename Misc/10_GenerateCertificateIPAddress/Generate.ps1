Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $IpAddress
)

$version = "1.0.2u"

if ($global:IsWindows)
{
   $os = "windows"
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
   $os = "ios"
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsLinux)
{
   $os = "linux"
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}

& "${openssl}" req -newkey rsa:2048 `
                   -days 3650 `
                   -nodes `
                   -x509 `
                   -subj "/C=/ST=/L=/O=/OU=/CN=${IpAddress}" `
                   -extensions v3_req `
                   -config "${opensslConfig}" `
                   -keyout server.key `
                   -out server.crt
Remove-Item ".rnd"