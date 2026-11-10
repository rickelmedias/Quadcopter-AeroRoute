<#
.SYNOPSIS
    Mostra (e opcionalmente grava) os logs de texto do firmware normal.

.DESCRIPTION
    Abre a porta serial USB da placa com DTR e RTS ativos e imprime o que
    chegar, até Ctrl+C. Sem -Port, lista as portas disponíveis. Se a placa reiniciar, execute de
    novo.

.PARAMETER Port
    Porta serial da placa (por exemplo, COM3).

.PARAMETER OutputPath
    Opcional. Arquivo de texto onde o log também é acrescentado, sem apagar
    o conteúdo anterior. A pasta é criada se necessário. Caminhos relativos
    partem da pasta atual do terminal.

.EXAMPLE
    .\tools\read_logs.ps1 -Port COM3 -OutputPath logs\bancada.txt

.EXAMPLE
    .\tools\read_logs.ps1
    Lista as portas disponíveis, sem abrir nenhuma delas.

.INPUTS
    Nenhuma. O script não recebe dados pelo pipeline.

.OUTPUTS
    System.String
    Sem Port, retorna os nomes das portas disponíveis. Durante a leitura,
    mostra os logs no terminal e, opcionalmente, acrescenta-os ao arquivo.

.NOTES
    Compatível com Windows PowerShell 5.1. O arquivo usa UTF-8 com BOM para
    preservar os acentos nessa versão.

    Feche outros terminais que ocupem a mesma COM. Ctrl+C encerra a leitura
    e o bloco finally libera a porta e fecha o arquivo.
#>
param(
    [string]$Port,
    [string]$OutputPath
)
$ErrorActionPreference = 'Stop'
if (-not $Port) {
    Write-Host 'Portas disponiveis:'
    [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    Write-Host 'Use: .\tools\read_logs.ps1 -Port COM5 [-OutputPath logs\bancada.txt]'
    return
}
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.Handshake = 'None'
$serial.DtrEnable = $true
$serial.RtsEnable = $true
$writer = $null
try {
    if ($OutputPath) {
        $resolvedOutput = [System.IO.Path]::GetFullPath($OutputPath)
        [System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($resolvedOutput)) | Out-Null
        $writer = [System.IO.StreamWriter]::new($resolvedOutput, $true)
        $writer.AutoFlush = $true
    }
    $serial.Open()
    Write-Host "Lendo $Port. Ctrl+C encerra. Se a placa reiniciar, execute novamente."
    while ($true) {
        $chunk = $serial.ReadExisting()
        if ($chunk) {
            Write-Host -NoNewline $chunk
            if ($writer) { $writer.Write($chunk) }
        }
        Start-Sleep -Milliseconds 50
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
    if ($writer) { $writer.Dispose() }
}
