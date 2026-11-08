# Mesma rota do AG no PC e na Raspberry Pi Zero W (tools/pi/ensaios/assinatura_ag.py), 07/11/2026

## PC
máquina: x86_64  Python 3.14.7
quadrado 40 cm (ensaio): ordem 3 2 1  160.0 cm  0.14 s  sha256 52d2b0c4833b930fac2f85f401469eaad305aad96a33c2eb77728a987d9da835
10 pontos (benchmark, semente 2): ordem 8 6 5 4 1 2 3 10 9 7  1936.1 cm  0.16 s  sha256 9ffc58307e6801c8d918888b4852a298b79ff15c0eafc4fb7166e0fd189619d4
30 pontos (benchmark, semente 1): ordem 16 12 14 13 9 3 7 8 11 5 2 1 4 10 6 15 18 20 19 17 21 25 23 28 29 30 27 26 24 22  3083.7 cm  0.39 s  sha256 11f260a1e0b1b8f25464dc00545475634b865a6c97a5e56d5f670cd6a28a0dd6

## Raspberry Pi Zero W
máquina: armv6l  Python 3.13.5
quadrado 40 cm (ensaio): ordem 3 2 1  160.0 cm  9.70 s  sha256 52d2b0c4833b930fac2f85f401469eaad305aad96a33c2eb77728a987d9da835
10 pontos (benchmark, semente 2): ordem 8 6 5 4 1 2 3 10 9 7  1936.1 cm  11.14 s  sha256 9ffc58307e6801c8d918888b4852a298b79ff15c0eafc4fb7166e0fd189619d4
30 pontos (benchmark, semente 1): ordem 16 12 14 13 9 3 7 8 11 5 2 1 4 10 6 15 18 20 19 17 21 25 23 28 29 30 27 26 24 22  3083.7 cm  23.92 s  sha256 11f260a1e0b1b8f25464dc00545475634b865a6c97a5e56d5f670cd6a28a0dd6
