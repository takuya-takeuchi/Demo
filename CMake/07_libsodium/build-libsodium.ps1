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
   $Configuration
)

$current = $PSScriptRoot

# update submodule
git submodule update --init --recursive .

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

$target = "libsodium"

$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir
if ($global:IsWindows)
{
    $build_dir = Join-Path ${sourceDir} builds | `
                 Join-Path -ChildPath msvc | `
                 Join-Path -ChildPath vs2019
    Push-Location $build_dir

    $MSBuild = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\MSBuild\Current\Bin\MSBuild.exe"
    if (!(Test-Path ${MSBuild}))
    {
       $MSBuild = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\MSBuild\Current\Bin\MSBuild.exe"
       if (!(Test-Path ${MSBuild}))
       {
          $MSBuild = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe"
       }
    }
    
    # & ${MSBuild} "libsodium.sln" "/m" "/t:clean;rebuild" "/p:Configuration=Dyn${Configuration};Platform=${arch}"
    & ${MSBuild} "libsodium.sln" "/m" "/t:clean;rebuild" "/p:Configuration=Static${Configuration};Platform=x64"    

    $build_dir = Join-Path ${sourceDir} bin | `
                 Join-Path -ChildPath x64 | `
                 Join-Path -ChildPath ${Configuration} | `
                 Join-Path -ChildPath v142

    # lib
    $dst = Join-Path $installDir lib
    New-Item $dst -Force -ItemType Directory | Out-Null
    Copy-Item ${build_dir}/* $dst -Force -Recurse
    # include
    $dst = Join-Path $installDir include
    New-Item $dst -Force -ItemType Directory | Out-Null
    Copy-Item ${include_dir}/*.h $dst -Force -Recurse
    
    $build_dir = Join-Path ${sourceDir} src | `
                 Join-Path -ChildPath libsodium
    Copy-Item ${build_dir}/include/sodium $dst -Recurse -Force

    Pop-Location
}
else
{
    chmod +x *.sh
    make clean 
    ./autogen.sh
    ./configure --disable-dependency-tracking --prefix=$PWD/.libsodium-build --with-pic="yes"
    make -j8
}
Pop-Location
