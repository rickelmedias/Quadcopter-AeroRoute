"""Compilação ARM reproduzível a partir dos fontes, sem usar objetos antigos do CubeIDE.

Uso (no Windows, com o STM32CubeIDE instalado):

    python tools/build.py              # build/Release (firmware normal)
    python tools/build.py --debug      # build/Debug
    python tools/build.py --capture    # build/Capture (ensaio de captura)
    python tools/build.py --gcc "C:\\caminho\\arm-none-eabi-gcc.exe"

Gera ELF, HEX, BIN, mapa de memória e um manifest.json com o hash dos fontes e
dos artefatos. Não grava a placa.
"""
import argparse
import concurrent.futures
from pathlib import Path
import subprocess
import shutil
import hashlib
import json

#: Raiz do projeto.
ROOT = Path(__file__).resolve().parents[1]
def main():
    """Compila todos os fontes, liga e gera os artefatos da variante pedida.

    A variante (Release, Debug, Capture ou CaptureDebug) define a pasta de
    saída. O prefixo do hash dos fontes vira CAP_FIRMWARE_ID, e --capture
    define CAPTURE_BUILD. Encerra com erro se a compilação falhar.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--gcc', help='Path to arm-none-eabi-gcc.exe')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--capture', action='store_true', help='Binary capture, text logs off; separate output directory')
    args = parser.parse_args()
    gcc = args.gcc or shutil.which('arm-none-eabi-gcc')
    if not gcc:
        candidates = list(Path('C:/ST').glob('*/STM32CubeIDE/plugins/*gnu-tools*/tools/bin/arm-none-eabi-gcc.exe'))
        if not candidates: parser.error('Pass --gcc with your ARM compiler path')
        gcc = str(sorted(candidates)[-1])
    variant = ('CaptureDebug' if args.debug else 'Capture') if args.capture else ('Debug' if args.debug else 'Release')
    out = ROOT/'build'/variant
    out.mkdir(parents=True, exist_ok=True)
    includes = ['Core/Inc','App/Inc','Drivers/STM32F4xx_HAL_Driver/Inc',
        'Drivers/STM32F4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Include',
        'Drivers/CMSIS/Device/ST/STM32F4xx/Include',
        'Middlewares/Third_Party/FreeRTOS/Source/include',
        'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2',
        'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F',
        'Middlewares/Third_Party/mavlink','USB_DEVICE/App','USB_DEVICE/Target',
        'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
        'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc']
    patterns = ['Core/Src/*.c','Core/Startup/*.s','App/Src/**/*.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/*.c','USB_DEVICE/**/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c',
        'Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c',
        'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c']
    sources = sorted({p for pattern in patterns for p in ROOT.glob(pattern) if 'template' not in p.name})
    inputs = set(sources)
    for inc in includes:
        inputs.update((ROOT/inc).rglob('*.h'))
    inputs.update([ROOT/'tools/build.py', ROOT/'STM32F411CEUX_FLASH.ld'])
    fingerprint = hashlib.sha256(variant.encode())
    for path in sorted(inputs):
        fingerprint.update(path.relative_to(ROOT).as_posix().encode()+b'\0')
        fingerprint.update(path.read_bytes())
    source_hash = fingerprint.hexdigest()
    arch = ['-mcpu=cortex-m4','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-mthumb']
    options = arch + ['-std=gnu11','-DUSE_HAL_DRIVER','-DSTM32F411xE','-g3',
        '-O0' if args.debug else '-Os','-ffunction-sections','-fdata-sections','-Wall','-Wextra',
        '-Wno-unused-parameter','-fstack-usage','--specs=nano.specs']
    options += ['-I'+str(ROOT/p) for p in includes]
    options += ['-DCAP_FIRMWARE_ID="'+source_hash[:24]+'"']
    if args.capture:
        options += ['-DCAPTURE_BUILD=1']
    def compile_one(src):
        """Compila um fonte; devolve (caminho do objeto, resultado do processo)."""
        obj = out/('obj/'+src.relative_to(ROOT).as_posix()+'.o')
        obj.parent.mkdir(parents=True,exist_ok=True)
        result = subprocess.run([gcc,*options,'-c',str(src),'-o',str(obj)],capture_output=True,text=True)
        return obj,result
    objects=[]
    failed=False
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for obj,result in pool.map(compile_one,sources):
            if result.stdout or result.stderr: print(result.stdout+result.stderr)
            failed |= result.returncode != 0
            objects.append(obj)
    if failed: raise SystemExit('Compilation failed')
    rsp=out/'objects.rsp'
    rsp.write_text('\n'.join('"'+str(p).replace('\\','/')+'"' for p in objects))
    elf=out/'DroneController.elf'
    subprocess.run([gcc,*arch,'@'+str(rsp),'-T'+str(ROOT/'STM32F411CEUX_FLASH.ld'),
        '--specs=nosys.specs','--specs=nano.specs','-Wl,--gc-sections',
        '-Wl,-Map='+str(out/'DroneController.map'),'-Wl,--start-group','-lc','-lm','-Wl,--end-group',
        '-o',str(elf)],check=True)
    suffix = '.exe' if str(gcc).endswith('.exe') else ''
    for fmt,ext in [('ihex','hex'),('binary','bin')]:
        subprocess.run([str(Path(gcc).with_name('arm-none-eabi-objcopy'+suffix)),'-O',fmt,str(elf),str(elf.with_suffix('.'+ext))],check=True)
    subprocess.run([str(Path(gcc).with_name('arm-none-eabi-size'+suffix)),str(elf)],check=True)
    manifest = {'variant': variant, 'firmware_id': source_hash[:24],
                'source_sha256': source_hash, 'compiler': str(gcc),
                'artifacts_sha256': {elf.with_suffix('.'+ext).name:
                    hashlib.sha256(elf.with_suffix('.'+ext).read_bytes()).hexdigest()
                    for ext in ('elf','hex','bin')}}
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'Built {len(sources)} sources: {elf}')
if __name__ == '__main__': main()
