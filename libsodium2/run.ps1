$target = Join-Path $PSScriptRoot "Test"
$password = "DifCi8mBLkzC4Qump6LfDci7aMTp4NWF"
$inputToEncrypt = "Lenna.png"
$encryptedFile = "Lenna_encrypted.enc"
$outputFile = "Lenna_decrypted.png"

& ${target} e ${password} ${inputToEncrypt} ${encryptedFile}
& ${target} d ${password} ${encryptedFile} ${outputFile}