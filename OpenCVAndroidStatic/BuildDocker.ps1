$current = Get-Location

cd docker
docker build -t demo-build-opencv-android .

Set-Location $current
