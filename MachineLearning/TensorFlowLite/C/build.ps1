#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $TensorflowVersion,

   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $BuildSystem
)

$buildSystems = @(
    "cmake",
    "bazel"
)

$BuildSystem = $BuildSystem.ToLower()
if ($buildSystems.Contains($BuildSystem) -eq $False)
{
   $candidate = $buildSystems -join "/"
   Write-Host "[Error] Specify build system [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot
$sciprtDir = Join-Path $current "scripts"
$script = Join-Path $sciprtDir "build-${BuildSystem}.ps1"
if (!(Test-Path($script)))
{
   Write-Host "[Error] ${script} is missing" -ForegroundColor Red
   exit -1
}

if ($BuildSystem -eq "bazel")
{
   # https://www.tensorflow.org/install/source_windows#gpu
   # python version should be decided by tensorflow version
   $arguments = @()
   $arguments += New-Object PSObject -Property @{ Version = "v2.5.3";  BazelVersion = "3.7.2"; PythonVersion = "3.8.9"; PythonIntegerVersion = "38"; CPythonVersion = "3.8"; }
   $arguments += New-Object PSObject -Property @{ Version = "v2.10.1"; BazelVersion = "5.1.1"; PythonVersion = "3.8.9"; PythonIntegerVersion = "38"; CPythonVersion = "3.8"; }
   $arguments += New-Object PSObject -Property @{ Version = "v2.11.1"; BazelVersion = "5.3.0"; PythonVersion = "3.8.9"; PythonIntegerVersion = "38"; CPythonVersion = "3.8"; }
   $arguments += New-Object PSObject -Property @{ Version = "v2.12.1"; BazelVersion = "5.3.0"; PythonVersion = "3.8.9"; PythonIntegerVersion = "38"; CPythonVersion = "3.8"; }
   
   $targetArgument = $arguments | Where-Object { $_.Version -eq $TensorflowVersion } | Select-Object -First 1
   if (!($targetArgument))
   {
      $candidate = ($arguments | Select-Object -ExpandProperty Version) -join "/"
      Write-Host "[Error] Specify Tensorflow version '${TensorflowVersion}' is not supported. Supported versions are [${candidate}]" -ForegroundColor Red
      exit -1
   }
   
   & "${script}" -Configuration $Configuration `
                 -Version $TensorflowVersion `
                 -BazelVersion $targetArgument.BazelVersion `
                 -PythonVersion $targetArgument.PythonVersion `
                 -PythonIntegerVersion $targetArgument.PythonIntegerVersion `
                 -CPythonVersion $targetArgument.CPythonVersion
}
elseif ($BuildSystem -eq "cmake")
{

}