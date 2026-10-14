<#
.SYNOPSIS
    Console do teste de motores: mostra o log da placa e envia comandos.

.DESCRIPTION
    Abre a porta USB da placa com DTR ligado (exigido para armar), imprime o
    que chegar e grava tudo em -OutputPath, se informado. O que for digitado
    é enviado quando se aperta Enter. Enter numa linha vazia PARA os motores.
    Ctrl+C fecha a porta, o que também para e desarma os motores.

    Use só com o firmware da branch motors e SEM HÉLICES.

.PARAMETER Port
    Porta serial da placa (por exemplo, COM5). Sem ela, lista as portas.

.PARAMETER OutputPath
    Opcional. Arquivo onde o log é acrescentado.

.EXAMPLE
    .\tools\motor_console.ps1 -Port COM5 -OutputPath logs\motores.txt
#>
param(
    [string]$Port,
    [string]$OutputPath
)
$ErrorActionPreference = 'Stop'
if (-not $Port) {
    Write-Host 'Portas disponiveis:'
    [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    Write-Host 'Use: .\tools\motor_console.ps1 -Port COM5 [-OutputPath logs\motores.txt]'
    return
}
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.Handshake = 'None'
$serial.DtrEnable = $true
$serial.RtsEnable = $true
$serial.NewLine = "`n"
$writer = $null
$typed = ''
try {
    if ($OutputPath) {
        $full = [System.IO.Path]::GetFullPath($OutputPath)
        [System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($full)) | Out-Null
        $writer = [System.IO.StreamWriter]::new($full, $true)
        $writer.AutoFlush = $true
    }
    $serial.Open()
    Write-Host "Conectado em $Port. Digite help e Enter. Enter vazio PARA. Ctrl+C sai."
    while ($true) {
        $chunk = $serial.ReadExisting()
        if ($chunk) {
            Write-Host -NoNewline $chunk
            if ($writer) { $writer.Write($chunk) }
        }
        while ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            if ($key.Key -eq 'Enter') {
                $serial.WriteLine($typed)
                Write-Host "`n>>> $typed"
                $typed = ''
            } elseif ($key.Key -eq 'Backspace') {
                if ($typed.Length -gt 0) { $typed = $typed.Substring(0, $typed.Length - 1) }
            } elseif ($key.KeyChar -ne [char]0) {
                $typed += $key.KeyChar
            }
        }
        Start-Sleep -Milliseconds 20
    }
}
finally {
    if ($serial.IsOpen) {
        try { $serial.WriteLine('stop') } catch {}
        $serial.Close()
    }
    if ($writer) { $writer.Dispose() }
}
