Param([Parameter(
      Mandatory=$True,
      Position = 1
      )][string]
      $serverUrl
)

$current = $PSScriptRoot
$version = "7.4.0"

if (!(Test-Path("openapi-generator-cli-${version}.jar")))
{
    Invoke-WebRequest -OutFile "openapi-generator-cli-${version}.jar" "https://repo1.maven.org/maven2/org/openapitools/openapi-generator-cli/${version}/openapi-generator-cli-${version}.jar"
}

$outputDir = Join-Path $current "openapi"
& java -jar "openapi-generator-cli-${version}.jar" generate `
       -i ${serverUrl}/swagger/v1/swagger.json `
       -g cpp-restsdk `
       -o "${outputDir}" `
       -c config.json
    #    --apiPackage=demo.client.api