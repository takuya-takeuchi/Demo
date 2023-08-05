#***************************************
#Arguments
#***************************************
Param
(
)

$libraryName = "cpuinfo"

$current = $PSScriptRoot
$installDir = Join-Path $current install

$TargetArray =
@(
   "osx",
   "iphoneos",
   "iphonesimulator"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64"
)

$vars = ""
foreach ($target in $TargetArray)
{
    $buildTargets = @()
    foreach ($architecture in $ArchitectureArray)
    {
        $libraryPath = Join-Path $installDir $target | `
                       Join-Path -ChildPath $architecture | `
                       Join-Path -ChildPath "lib" | `
                       Join-Path -ChildPath "libcpuinfo.dylib"
        if (!(Test-Path($libraryPath)))
        {
            continue
        }

        $headerDir = Join-Path $installDir $target | `
                     Join-Path -ChildPath $architecture | `
                     Join-Path -ChildPath "include"
        if (!(Test-Path($headerDir)))
        {
            continue
        }
        
        $buildTargets += New-Object PSObject -Property @{ LibraryPath = "${libraryPath}"; HeaderDir = "${headerDir}" }
    }

    # https://developer.apple.com/forums/thread/666335
    # Shall merge multiple binaries to one binary for per target
    if ($buildTargets.Length -gt 1)
    {
        $args = ""
        foreach ($buildTarget in $buildTargets)
        {
            $libraryPath = $buildTarget.LibraryPath
            $args += " ${libraryPath}"
        }

        $libraryPath = Join-Path $installDir $target | `
                       Join-Path -ChildPath "lib${libraryName}.dylib"

        $proc = Start-Process -FilePath lipo -ArgumentList "-create -output ${libraryPath} ${args}" -NoNewWindow -PassThru
        $proc.WaitForExit()

        $vars += " -library ${libraryPath} -headers ${headerDir}"
    }
    elseif ($buildTargets.Length -eq 1)
    {
        foreach ($buildTarget in $buildTargets)
        {
            $libraryPath = $buildTarget.LibraryPath
            $headerDir = $buildTarget.HeaderDir
            $vars += " -library ${libraryPath} -headers ${headerDir}"
        }
    }
}

if (!($vars))
{
    exit
}

$frameworkDir = Join-Path ${installDir} "${libraryName}.xcframework"
$proc = Start-Process -FilePath xcodebuild -ArgumentList "-create-xcframework ${vars} -output ${frameworkDir}" -NoNewWindow -PassThru
$proc.WaitForExit()