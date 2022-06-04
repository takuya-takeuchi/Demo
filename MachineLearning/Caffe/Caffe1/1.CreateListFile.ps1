[System.IO.Directory]::SetCurrentDirectory((Get-Location -PSProvider FileSystem).Path)

# It must specify relative path
$trains = "figs_0",
          "figs_1",
          "figs_2",
          "figs_3",
          "figs_4",
          "figs_5",
          "figs_6";
$tests = "figs_7";

$patterns = "A",
            "L",
            "R",
            "T",
            "W";

$output_listfile_train = "train.txt"
$output_listfile_test = "test.txt"


$sb=New-Object System.Text.StringBuilder

foreach($e in $trains)
{
    $target = $e + "\*.*"
    foreach($d in Get-ChildItem $target -include *.txt)
    {
        $path = Resolve-Path -Relative $d
        $p = [System.IO.File]::ReadAllLines($path)[1].Replace("Class: ","")
        [void]$sb.AppendLine([String]::Format("{0} {1}",$path.Replace(".txt",".png"), $patterns.IndexOf($p)))
    }
}

$Utf8NoBomEncoding = New-Object System.Text.UTF8Encoding($False)
[System.IO.File]::WriteAllLines($output_listfile_train, $sb.ToString(), $Utf8NoBomEncoding)

$sb.Length = 0;

foreach($e in $tests)
{
    $target = $e + "\*.*"
    foreach($d in Get-ChildItem $target -include *.txt)
    {
        $path = Resolve-Path -Relative $d
        $p = [System.IO.File]::ReadAllLines($path)[1].Replace("Class: ","")
        [void]$sb.AppendLine([String]::Format("{0} {1}",$path.Replace(".txt",".png"), $patterns.IndexOf($p)))
    }
}

[System.IO.File]::WriteAllLines($output_listfile_test, $sb.ToString(), $Utf8NoBomEncoding)
