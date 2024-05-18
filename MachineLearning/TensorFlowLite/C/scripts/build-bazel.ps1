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
   $Version,

   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $BazelVersion,

   [Parameter(
   Mandatory=$True,
   Position = 4
   )][string]
   $PythonVersion,

   [Parameter(
   Mandatory=$True,
   Position = 5
   )][string]
   $PythonIntegerVersion,

   [Parameter(
   Mandatory=$True,
   Position = 6
   )][string]
   $CPythonVersion
)

$current = Split-Path $PSScriptRoot -Parent

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "tensorflow"
$version = $Version
$lang = "c"
$buildSystem = "bazel"
$bazel_version = $BazelVersion
# https://www.tensorflow.org/install/source_windows#gpu
$python_version = $PythonVersion
$python_major_version = $PythonIntegerVersion
$cpython_version = $CPythonVersion
$visualStudio = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $lang | `
            Join-Path -ChildPath $buildSystem
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $lang | `
              Join-Path -ChildPath $buildSystem | `
              Join-Path -ChildPath $Configuration | `
              Join-Path -ChildPath $version
$installBazelDir = Join-Path $current install | `
                   Join-Path -ChildPath $os | `
                   Join-Path -ChildPath $buildSystem | `
                   Join-Path -ChildPath $bazel_version
$installPythonDir = Join-Path $current install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath python | `
                    Join-Path -ChildPath $python_version
$installCPythonDir = Join-Path $current install | `
                     Join-Path -ChildPath $os | `
                     Join-Path -ChildPath cpython | `
                     Join-Path -ChildPath $cpython_version
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath $buildSystem

# download bazel
if ($global:IsWindows)
{
    $bazel_url_filename = "bazel-${bazel_version}-windows-x86_64.exe"
    $url = "https://github.com/bazelbuild/bazel/releases/download/${bazel_version}/${bazel_url_filename}"
    $bazel_filename = "bazel.exe"
    $bazel = Join-Path $installBazelDir $bazel_filename
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

if (!(Test-Path("${bazel}")))
{
    Write-Host "Download ${buildSystem} from ${url}" -ForegroundColor Blue
    New-Item -Type Directory $installBazelDir -Force | Out-Null
    Invoke-WebRequest "${url}" -OutFile "${bazel}"
}

if ($global:IsWindows)
{
    $python_url_filename = "python-${python_version}-embed-amd64.zip"
    $url = "https://www.python.org/ftp/python/${python_version}/${python_url_filename}"
    $python_filename = "python.exe"
    $python = Join-Path $installPythonDir "python.exe"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

if (!(Test-Path("${python}")))
{    
    Write-Host "Download embed python from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${python_url_filename}"

    New-Item -Type Directory $installPythonDir -Force | Out-Null
    Expand-Archive -Path "${python_url_filename}" -DestinationPath $installPythonDir
    Remove-Item "${python_url_filename}" -Force | Out-Null

    # replace `#import site`
    $pth = Join-Path $installPythonDir "python${python_major_version}._pth"
    $content = Get-Content -Path $pth -Raw
    $updatedContent = $content -replace "#import site", "import site"
    Set-Content -Path $pth -Value $updatedContent

    # expand python3X.zip to Lib
    $zip = Join-Path $installPythonDir "python${python_major_version}.zip"
    $libDir = Join-Path $installPythonDir Lib
    Expand-Archive -Path "${zip}" -DestinationPath $libDir

    # build cpython to get include and libs
    $buildCPythonDir = Join-Path $installCPythonDir "PCbuild"
    $bat = Join-Path $buildCPythonDir "build.bat"
    if (!(Test-Path $installCPythonDir))
    {
        git clone -b $cpython_version "https://github.com/python/cpython" $installCPythonDir
    }

    Push-Location $buildCPythonDir | Out-Null
    git checkout $cpython_version
    $args = "-e -p x64"
    Start-Process -FilePath "cmd.exe" -ArgumentList "/c", ${bat}, $args -Wait
    Start-Process -FilePath "cmd.exe" -ArgumentList "/c", ${bat}, $args -Wait
    Pop-Location

    # copy inlcude dir
    $includeDir = Join-Path $installCPythonDir include
    Copy-Item "${includeDir}" "${installPythonDir}" -Force -Recurse
    
    # copy libs
    $libDir = Join-Path $installPythonDir "libs"
    New-Item -Type Directory $libDir -Force | Out-Null
    $cpythonLibs = @( "_tkinter.lib", "python3.lib", "python${python_major_version}.lib" )
    foreach ($lib in $cpythonLibs)
    {
        $src = Join-Path $buildCPythonDir "amd64" | Join-Path -ChildPath $lib
        $dst  = Join-Path $libDir $lib
        Copy-Item "${src}" "${dst}" -Force
    }
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir
git checkout $version

if ($global:IsWindows)
{
    $env:PATH="${env:PATH};$installBazelDir"

    $env:PYTHON_BIN_PATH="${python}"
    $env:PYTHON_LIB_PATH=(Join-Path "${installPythonDir}" Lib | Join-Path -ChildPath "site-packages")
    $env:PYTHON_DIRECTORY="${installPythonDir}"
    $PYTHON_SCRIPTS=(Join-Path "${installPythonDir}" Scripts)
    $env:PATH="${PYTHON_SCRIPTS};${env:PATH}"

    # install pip
    $pip = Join-Path "${PYTHON_SCRIPTS}" "pip.exe"
    if (!(Test-Path($pip)))
    {
        Write-Host "Install pip..." -ForegroundColor Blue
        Invoke-WebRequest https://bootstrap.pypa.io/get-pip.py -OutFile get-pip.py
        & "${python}" get-pip.py
    }
    # install numpy
    $numpy = Join-Path "${installPythonDir}" "Lib" | Join-Path -ChildPath "site-packages" | Join-Path -ChildPath numpy
    if (!(Test-Path($numpy)))
    {
        Write-Host "Install numpy (pip: ${pip})..." -ForegroundColor Blue
        & "${python}" "${pip}" install numpy
    }

    # skip question
    $env:USE_DEFAULT_PYTHON_LIB_PATH="1"
    $env:TF_NEED_ROCM="0"
    $env:TF_NEED_CUDA="0"
    $env:CC_OPT_FLAGS="/arch:AVX"
    $env:TF_OVERRIDE_EIGEN_STRONG_INLINE="Y"
    $env:TF_SET_ANDROID_WORKSPACE="N"
    
    Write-Host "Launch configure.py (PYTHON_BIN_PATH: $env:PYTHON_BIN_PATH)..." -ForegroundColor Blue
    & "C:\msys64\usr\bin\bash.exe" -c "`$PYTHON_BIN_PATH configure.py"    

    function CallVisualStudioDeveloperConsole()
    {
        cmd.exe /c "call `"${visualStudio}\VC\Auxiliary\Build\vcvars64.bat`" && set > %temp%\vcvars.txt"
        Get-Content "${env:temp}\vcvars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }

    CallVisualStudioDeveloperConsole
    $env:BAZEL_VC="${visualStudio}\VC"

    $bazel = "/${bazel}".Replace("`\", "/").Replace(":", "")
    $workspace = "/${sourceDir}".Replace("`\", "/").Replace(":", "")
    Write-Host "Launch bazel.exe (1st)..." -ForegroundColor Blue
    & "C:\msys64\usr\bin\bash.exe" -c "source /etc/profile; cd ${workspace}; ${bazel} build --config=monolithic -c opt //tensorflow/lite/c:tensorflowlite_c"

    $patchRootDir = Join-Path $current "patches" | Join-Path -ChildPath $version
    if (!(Test-Path($patchRootDir)))
    {
        Write-Host "No patches..." -ForegroundColor Blue
    }
    else
    {
        Write-Host "Apply patches..." -ForegroundColor Blue
        $patchDir = Join-Path $patchRootDir "*"
        $destDir = Join-Path $sourceDir "bazel-tensorflow"
        Write-Host "Copy-Item -Force ${patchDir} ${destDir}" -ForegroundColor Blue
        Copy-Item -Force -Recurse $patchDir $destDir
    }

    Write-Host "Launch bazel.exe (2nd)..." -ForegroundColor Blue
    & "C:\msys64\usr\bin\bash.exe" -c "source /etc/profile; cd ${workspace}; ${bazel} build --config=monolithic -c opt //tensorflow/lite/c:tensorflowlite_c"

    Write-Host "Shutdown bazel.exe..." -ForegroundColor Blue
    & "C:\msys64\usr\bin\bash.exe" -c "source /etc/profile; cd ${workspace}; ${bazel} shutdown"

    Write-Host "Copy artifacts..." -ForegroundColor Blue
    $srcDir = Join-Path $sourceDir "bazel-bin" | Join-Path -ChildPath "tensorflow" | Join-Path -ChildPath "lite" | Join-Path -ChildPath "c" |Join-Path -ChildPath "*\"
    $dstDir = Join-Path $installDir "lib"
    Copy-Item -Force -Recurse $srcDir $dstDir

    Write-Host "Clean up artifacts..." -ForegroundColor Blue
    $allowedExtensions = @(".lib", ".dll")
    Get-ChildItem -Path $dstDir -File -Recurse | Where-Object { $extension = $_.Extension.ToLower()
                                                              -not ($allowedExtensions -contains $extension) } |
                                                 ForEach-Object { Remove-Item -Path $_.FullName -Force }
    # remove empty directories
    do {
        $targets = Get-ChildItem -Path $dstDir -Directory -Recurse | Where-Object { $_.GetFileSystemInfos().Count -eq 0 }
        if ($targets.length -eq 0)
        {
            break
        }
        $targets | ForEach-Object { Remove-Item -Path $_.FullName -Force -Recurse }
    } while ($true)

    switch ($Version)
    {
        "v2.5.3"
        {
            $includes = @()
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api.h";              Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_experimental.h"; Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_types.h";        Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "common.h";             Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
            $includeInstallDir = Join-Path $installDir include
            foreach ($include in $includes)
            {
                $includeDestDir = Join-Path $includeInstallDir tensorflow | `
                                    Join-Path -ChildPath $include.Target
                New-Item -Type Directory $includeDestDir -Force | Out-Null

                $includeSourceDir = Join-Path $sourceDir tensorflow | `
                                    Join-Path -ChildPath $include.Source
                $src = Join-Path $includeSourceDir $include.Name
                Copy-Item "${src}" "${includeDestDir}" -Force
            }
        }
        "v2.10.1"
        {
            $includes = @()
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api.h";              Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_experimental.h"; Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "c_api_types.h";        Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite\c";        Name = "common.h";             Target = "lite\c"; }
            $includes += New-Object PSObject -Property @{ Source = "lite";          Name = "builtin_ops.h";        Target = "lite"; }
            $includeInstallDir = Join-Path $installDir include
            foreach ($include in $includes)
            {
                $includeDestDir = Join-Path $includeInstallDir tensorflow | `
                                    Join-Path -ChildPath $include.Target
                New-Item -Type Directory $includeDestDir -Force | Out-Null

                $includeSourceDir = Join-Path $sourceDir tensorflow | `
                                    Join-Path -ChildPath $include.Source
                $src = Join-Path $includeSourceDir $include.Name
                Copy-Item "${src}" "${includeDestDir}" -Force
            }
        }
    }
}
elseif ($global:IsMacOS)
{
}
elseif ($global:IsLinux)
{
}
Pop-Location