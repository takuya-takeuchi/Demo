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

$directories = @(
   "CA"
   "ICA"
   "Server"
)

foreach ($directory in $directories)
{
   $root = Join-Path $current pki | Join-Path -ChildPath $directory
   New-Item -Type Directory -Force $root | Out-Null
   Push-Location $root
   New-Item -Type Directory -Force certs | Out-Null
   New-Item -Type Directory -Force db | Out-Null
   # chmod 700 private
   $index = Join-Path db index
   New-Item -Type File -Force $index | Out-Null
   $serial = Join-Path db serial
   & "${openssl}" rand -hex 16 | Out-File -FilePath "${serial}"
   $crlnumber = Join-Path db crlnumber
   & "${openssl}" rand -hex 16 | Out-File -FilePath "${crlnumber}"
   Pop-Location
}