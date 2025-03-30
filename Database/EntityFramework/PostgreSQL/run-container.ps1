$current = $PSScriptRoot

$dataDir = Join-Path $current data
New-Item -Type Directory $dataDir 2>&1 > $null

docker run --name postgres `
           -e POSTGRES_PASSWORD=postgres `
           -p 5432:5432 `
           -v "${dataDir}:/var/lib/postgresql/data" `
           -d postgres:17.4