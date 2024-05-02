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

$settings = @()
$settings += New-Object PSObject -Property @{Target = "Root CA";         Config = "root-ca.conf";          RootDirectory="CA";     Name = "rca";    Parent=""; }
$settings += New-Object PSObject -Property @{Target = "Intermediate CA"; Config = "intermediate-ca.conf";  RootDirectory="ICA";    Name = "ica";    Parent="CA"; }
$settings += New-Object PSObject -Property @{Target = "Server";          Config = "server.conf";           RootDirectory="Server"; Name = "server"; Parent="ICA"; }

# check config files
foreach ($setting in $settings)
{
   $config = Join-Path $current $setting.Config
   if (!(Test-Path("${config}")))
   {
      Write-Host "'${config}' is missing" -ForegroundColor Red
      exit
   }
}

foreach ($setting in $settings)
{
   $name = $setting.Name
   $target = $setting.Target
   Write-Host "`n[${target}]" -ForegroundColor Blue
   
   $config = Join-Path $current $setting.Config
   $root = Join-Path $current pki | Join-Path -ChildPath $setting.RootDirectory
   $privateKey = Join-Path $root private | Join-Path -ChildPath key.pem
   $csr = Join-Path $root csr.pem
   $crt = Join-Path $root crt.pem
   $certificate = Join-Path $root "${name}.crt"
   
   Write-Host "Create private key..." -ForegroundColor Blue
   & "${openssl}" genrsa -out "${privateKey}"

   Write-Host "Create certificate signing request..." -ForegroundColor Blue
   & "${openssl}" req -config "${config}" -new -key "${privateKey}" -out "${csr}"


   $parent = $setting.Parent
   if ($parent -eq "")
   {
      Write-Host "Do Self-Sign certificate signing request..." -ForegroundColor Blue
      & "${openssl}" ca -batch -config "${config}" -selfsign `
                        -in "${csr}" -out "${crt}" `
                        -extensions req_ext
   }
   else
   {
      $parent = Join-Path $current pki | Join-Path -ChildPath $parent
      $parentCrt = Join-Path $parent crt.pem
      $parentPrivateKey = Join-Path $parent private | Join-Path -ChildPath key.pem
      
      if (!(Test-Path("${parentCrt}")))
      {
         Write-Host "'${parentCrt}' is missing" -ForegroundColor Red
         exit
      }
      if (!(Test-Path("${parentPrivateKey}")))
      {
         Write-Host "'${parentPrivateKey}' is missing" -ForegroundColor Red
         exit
      }

      Write-Host "Do Sign certificate signing request..." -ForegroundColor Blue
      & "${openssl}" ca -batch -config "${config}" `
                        -in "${csr}" -out "${crt}" `
                        -keyfile "${parentPrivateKey}"-cert "${parentCrt}" `
                        -extensions req_ext
   }

   Write-Host "Create certificate..." -ForegroundColor Blue
   & "${openssl}" x509 -in "${crt}" -out "${certificate}"
}