"""Compilação ARM reproduzível a partir dos fontes, sem usar objetos antigos do CubeIDE.

Uso (Windows ou Linux, com o STM32CubeIDE instalado):

    python tools/build.py                     # build/Release
    python tools/build.py --debug             # build/Debug
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

    A variante (Release ou Debug) define a pasta de saída. Encerra com erro
    se a compilação falhar.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--gcc', help='Path to arm-none-eabi-gcc.exe')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument(
        '--sim',
        action='store_true',
        help='FLT_SIM=1: planta simulada no lugar do BNO/MTF, motores reais em zero',
    )
    parser.add_argument(
        '--no-cg-trim',
        action='store_true',
        help='LQG_CG_TRIM_ENABLE=0 (variante NoTrim): quatro motores com a mesma base, para comparar',
    )
    args = parser.parse_args()
    if args.no_cg_trim and (args.sim or args.debug):
        parser.error('--no-cg-trim só vale na variante Release')
    gcc = args.gcc or shutil.which('arm-none-eabi-gcc')
    if not gcc:
        candidates = list(
            Path('C:/ST').glob('*/STM32CubeIDE/plugins/*gnu-tools*/tools/bin/arm-none-eabi-gcc.exe')
        ) + list(Path('/opt/st').glob('*/plugins/*gnu-tools*/tools/bin/arm-none-eabi-gcc'))
        if not candidates:
            parser.error('Pass --gcc with your ARM compiler path')
        gcc = str(sorted(candidates)[-1])
    # A variante vai no HELLO (8 caracteres): Sim/DebugSim mostram na IHM que é FLT_SIM.
    variant = ('Debug' if args.debug else ('' if args.sim else 'Release')) + (
        'Sim' if args.sim else ''
    )
    if args.no_cg_trim:
        variant = 'NoTrim'  # no HELLO (8 caracteres): a IHM mostra que o trim do CG está desligado
    out = ROOT / 'build' / variant
    out.mkdir(parents=True, exist_ok=True)
    includes = [
        'Core/Inc',
        'App/Inc',
        'Drivers/STM32F4xx_HAL_Driver/Inc',
        'Drivers/STM32F4xx_HAL_Driver/Inc/Legacy',
        'Drivers/CMSIS/Include',
        'Drivers/CMSIS/Device/ST/STM32F4xx/Include',
        'Middlewares/Third_Party/FreeRTOS/Source/include',
        'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2',
        'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F',
        'USB_DEVICE/App',
        'USB_DEVICE/Target',
        'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
        'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc',
        'Middlewares/Third_Party/mavlink',
    ]
    patterns = [
        'Core/Src/*.c',
        'Core/Startup/*.s',
        'App/Src/**/*.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/*.c',
        'USB_DEVICE/**/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c',
        'Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c',
        'Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c',
        'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c',
    ]
    sources = sorted(
        {p for pattern in patterns for p in ROOT.glob(pattern) if 'template' not in p.name}
    )
    inputs = set(sources)
    for inc in includes:
        inputs.update((ROOT / inc).rglob('*.h'))
    inputs.update([ROOT / 'tools/build.py', ROOT / 'STM32F411CEUX_FLASH.ld'])
    fingerprint = hashlib.sha256(f'phase2-bench/{variant}'.encode())
    for path in sorted(inputs):
        fingerprint.update(path.relative_to(ROOT).as_posix().encode() + b'\0')
        fingerprint.update(path.read_bytes())
    source_hash = fingerprint.hexdigest()
    arch = ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-mthumb']
    options = arch + [
        '-std=gnu11',
        '-DUSE_HAL_DRIVER',
        '-DSTM32F411xE',
        # Identidade enviada no HELLO do bench-link: liga a sessão gravada ao manifest.
        f'-DAEROROUTE_FIRMWARE_ID="{source_hash[:24]}"',
        f'-DAEROROUTE_BUILD_VARIANT="{variant}"',
        f'-DFLT_SIM={1 if args.sim else 0}',
        f'-DLQG_CG_TRIM_ENABLE={0 if args.no_cg_trim else 1}',
        '-g3',
        '-O0' if args.debug else '-Os',
        '-ffunction-sections',
        '-fdata-sections',
        '-Wall',
        '-Wextra',
        '-Wno-unused-parameter',
        '-fstack-usage',
        '--specs=nano.specs',
    ]
    options += ['-I' + str(ROOT / p) for p in includes]

    def compile_one(src):
        """Compila um fonte; devolve (caminho do objeto, resultado do processo)."""
        obj = out / ('obj/' + src.relative_to(ROOT).as_posix() + '.o')
        obj.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [gcc, *options, '-c', str(src), '-o', str(obj)], capture_output=True, text=True
        )
        return obj, result

    objects = []
    failed = False
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for obj, result in pool.map(compile_one, sources):
            if result.stdout or result.stderr:
                print(result.stdout + result.stderr)
            failed |= result.returncode != 0
            objects.append(obj)
    if failed:
        raise SystemExit('Compilation failed')
    rsp = out / 'objects.rsp'
    rsp.write_text('\n'.join('"' + str(p).replace('\\', '/') + '"' for p in objects))
    elf = out / 'DroneController.elf'
    subprocess.run(
        [
            gcc,
            *arch,
            '@' + str(rsp),
            '-T' + str(ROOT / 'STM32F411CEUX_FLASH.ld'),
            '--specs=nosys.specs',
            '--specs=nano.specs',
            '-Wl,--gc-sections',
            '-Wl,-Map=' + str(out / 'DroneController.map'),
            '-Wl,--start-group',
            '-lc',
            '-lm',
            '-Wl,--end-group',
            '-o',
            str(elf),
        ],
        check=True,
    )
    suffix = '.exe' if str(gcc).endswith('.exe') else ''
    for fmt, ext in [('ihex', 'hex'), ('binary', 'bin')]:
        subprocess.run(
            [
                str(Path(gcc).with_name('arm-none-eabi-objcopy' + suffix)),
                '-O',
                fmt,
                str(elf),
                str(elf.with_suffix('.' + ext)),
            ],
            check=True,
        )
    subprocess.run([str(Path(gcc).with_name('arm-none-eabi-size' + suffix)), str(elf)], check=True)
    manifest = {
        'variant': variant,
        'profile': 'bench',
        'capabilities': {'manual_motors': True, 'usb_protocol': 'bench-link-v1'},
        'firmware_id': source_hash[:24],
        'source_sha256': source_hash,
        'compiler': str(gcc),
        'artifacts_sha256': {
            elf.with_suffix('.' + ext).name: hashlib.sha256(
                elf.with_suffix('.' + ext).read_bytes()
            ).hexdigest()
            for ext in ('elf', 'hex', 'bin')
        },
    }
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Built {len(sources)} sources: {elf}')


if __name__ == '__main__':
    main()
