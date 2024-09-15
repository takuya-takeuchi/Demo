$current = Get-Location

$work = Join-Path $current work
$opencv = Join-Path $work opencv

if (!(Test-Path $opencv))
{
    Set-Location $work
    git clone -b 3.4.10 https://github.com/opencv/opencv opencv
}

docker run --rm -v ${work}:/opt/work --workdir=/opt/work -it demo-build-opencv-android /bin/bash


Set-Location $currrent
