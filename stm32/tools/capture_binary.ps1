<#
.SYNOPSIS
    Grava a captura binária do firmware Capture pela porta serial USB.

.DESCRIPTION
    Cria um arquivo novo (nunca sobrescreve), abre a COM com DTR baixo, limpa
    o que houver na entrada e então ativa o DTR, o que inicia a sessão no
    firmware. Lê bytes crus, sem nenhuma conversão de texto, pelo tempo
    pedido. No fim baixa o DTR e tenta drenar os dados pendentes, continuando
    a leitura por até 3 s ou até passar 0,5 s sem dados. Grava ao lado do .bin
    um .bin.json com
    metadados e SHA-256 e, se existir, uma cópia do manifest do build Capture
    (.bin.firmware.json).

    Só há uma sessão por boot: reinicie a placa antes de cada gravação.

.PARAMETER Port
    Porta serial da placa (padrão COM3).

.PARAMETER OutputPath
    Arquivo binário de saída. Precisa ainda não existir; a pasta é criada
    automaticamente. Caminhos relativos partem da pasta atual do terminal.

.PARAMETER Seconds
    Duração da gravação, de 1 a 3600 s (padrão 30).

.PARAMETER SelfTest
    Não abre a porta: só confere que o caminho de cópia preserva os 256
    valores de byte.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\tools\capture_binary.ps1 -Port COM3 -Seconds 45 -OutputPath captures\ensaio03.bin

.EXAMPLE
    .\tools\capture_binary.ps1 -SelfTest

.INPUTS
    Nenhuma. O script não recebe dados pelo pipeline.

.OUTPUTS
    Nenhuma saída de dados pelo pipeline. Os bytes são gravados no arquivo;
    as mensagens de andamento aparecem no terminal.

.NOTES
    Compatível com Windows PowerShell 5.1. O arquivo usa UTF-8 com BOM para
    preservar os acentos nessa versão.

    Use o firmware build/Capture/DroneController.hex e feche Tera Term ou
    qualquer outro programa que esteja usando a mesma porta. DTR é o sinal
    de controle que avisa ao firmware que o gravador está pronto; o arquivo
    é aberto antes de esse sinal ser ativado.

    Reinicie a placa antes de cada nova sessão. Não use read_logs.ps1 para
    gravar este fluxo: ele converte bytes em texto. Ctrl+C ou uma desconexão
    podem deixar o último registro incompleto; confira o arquivo com decoder
    e replay. O encerramento por Seconds permite tentar drenar a fila.

    A cópia do manifest identifica o build local, mas não verifica qual
    firmware está instalado na placa. O SelfTest verifica apenas a cópia
    binária em memória; não testa a porta, DTR ou os sensores.
#>
param(
    [string]$Port = 'COM3',
    [string]$OutputPath = 'captures/bancada.bin',
    [ValidateRange(1, 3600)][int]$Seconds = 30,
    [switch]$SelfTest
)
$ErrorActionPreference = 'Stop'

<#
.SYNOPSIS
    Copia um bloco de bytes de uma origem para um fluxo, sem conversão.
.DESCRIPTION
    Também é exercitada pelo SelfTest. Nunca use ReadExisting, StreamReader,
    Write-Host ou um Encoding para os bytes medidos.
.PARAMETER Reader
    Origem com método Read(byte[], int, int) (porta serial ou MemoryStream).
.PARAMETER Writer
    Fluxo de destino.
.PARAMETER Buffer
    Buffer de trabalho; define o tamanho máximo do bloco.
.OUTPUTS
    Quantidade de bytes copiados (0 ao atingir o fim de um fluxo). O timeout
    da porta serial é propagado para o chamador tratar.
#>
function Copy-BinaryBlock($Reader, [System.IO.Stream]$Writer, [byte[]]$Buffer) {
    $count = $Reader.Read($Buffer, 0, $Buffer.Length)
    if ($count -gt 0) { $Writer.Write($Buffer, 0, $count) }
    return $count
}
if ($SelfTest) {
    [byte[]]$expected = 0..255
    $inputStream = [System.IO.MemoryStream]::new($expected, $false)
    $outputStream = [System.IO.MemoryStream]::new()
    [byte[]]$testBuffer = New-Object byte[] 37
    try {
        while ((Copy-BinaryBlock $inputStream $outputStream $testBuffer) -gt 0) {}
        $actual = $outputStream.ToArray()
        if ($actual.Length -ne 256) { throw 'Binary self-test: wrong length' }
        for ($i = 0; $i -lt 256; $i++) {
            if ($actual[$i] -ne $expected[$i]) { throw "Binary self-test: changed byte $i" }
        }
        Write-Host 'PASS: all 256 byte values preserved, including sync 0xA5.'
    } finally { $inputStream.Dispose(); $outputStream.Dispose() }
    return
}

$resolved = [System.IO.Path]::GetFullPath($OutputPath)
[System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($resolved)) | Out-Null
# Recusa sobrescrever ou emendar uma sessão antiga.
$writer = [System.IO.FileStream]::new($resolved, [System.IO.FileMode]::CreateNew,
    [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.Handshake = 'None'
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 100
$serial.ReadBufferSize = 65536
[byte[]]$buffer = New-Object byte[] 8192
$started = [DateTime]::UtcNow
$total = 0L
$status = 'interrupted'
try {
    $manifest = Join-Path $PSScriptRoot '../build/Capture/manifest.json'
    if (Test-Path -LiteralPath $manifest) {
        Copy-Item -LiteralPath $manifest -Destination ($resolved + '.firmware.json')
    }
    $serial.Open()
    $serial.DiscardInBuffer() # o DTR ainda está baixo; o firmware não começou.
    $serial.DtrEnable = $true
    Write-Host "Capturando bytes de $Port por $Seconds s em $resolved"
    Write-Host 'Use firmware Capture e reinicie a placa antes de cada nova sessão.'
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        try { $total += Copy-BinaryBlock $serial $writer $buffer }
        catch [System.TimeoutException] { }
    }
    # Para os produtores, mantendo a porta aberta para drenar os bytes já na fila.
    $serial.DtrEnable = $false
    $drain = [System.Diagnostics.Stopwatch]::StartNew()
    $lastData = 0L
    while ($drain.ElapsedMilliseconds -lt 3000 -and
           ($drain.ElapsedMilliseconds - $lastData) -lt 500) {
        try {
            $n = Copy-BinaryBlock $serial $writer $buffer
            $total += $n
            if ($n -gt 0) { $lastData = $drain.ElapsedMilliseconds }
        } catch [System.TimeoutException] { }
    }
    $status = if ($total -gt 0) { 'saved' } else { 'empty' }
} finally {
    try {
        if ($serial.IsOpen) { $serial.DtrEnable = $false; $serial.Close() }
    } finally {
        $serial.Dispose()
        $writer.Dispose()
        @{
            port=$Port; started_utc=$started.ToString('o'); status=$status
            bytes=$total; requested_seconds=$Seconds
            sha256=(Get-FileHash -LiteralPath $resolved -Algorithm SHA256).Hash
        } | ConvertTo-Json | Set-Content -LiteralPath ($resolved + '.json') -Encoding UTF8
    }
}
if ($total -eq 0) { throw 'Nenhum byte recebido. Confira firmware Capture, COM e reinicie a placa.' }
Write-Host "Salvos $total bytes. Execute decode_capture.py e replay para verificar a sessão."
