$version = "1.0.2u"

$current = $PSScriptRoot
if ($global:IsWindows)
{
   $openssl = Join-Path $current openssl | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath windows | `
              Join-Path -ChildPath openssl.exe
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
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

Write-Host "Check server's certificate signing request..." -ForegroundColor Blue
& "${openssl}" req -text -noout -in server.csr