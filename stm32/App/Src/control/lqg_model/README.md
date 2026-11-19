# Modelo LQR + Kalman gerado pelo Simulink

Cópia **sem alterações** de `references/Cod_C_STM32_V2` (modelo `STM32`, versão
2.18, Simulink Coder 24.1, gerado em 2026-11-11 19:55). Não editar à mão: para
mudar ganhos ou modelo, regerar no MATLAB e copiar de novo estes seis arquivos.

| Item | Conteúdo |
| --- | --- |
| Passo | `STM32_step()` a Ts = 0,01 s |
| Estados (12) | `[x y z φ θ ψ u v w p q r]`, SI, NED/FRD, linearizado no hover |
| Entradas `In1` (4) | referência `[x_ref y_ref z_ref ψ_ref]` em m e rad (z para baixo) |
| Medidas `y` (9) | `[z φ θ ψ u v p q r]` |
| Saída `Out1` (4) | Δω de cada motor em rad/s em torno do hover (M1..M4) |
| Controle | `Out1 = Kfd·r − Kd·x̂`, com x̂ do estimador corrente (`KalmanGainM`) |
| Kalman | ganho estacionário `KalmanGainL`/`KalmanGainM`, predição com `A`,`B` |

SHA-256 dos arquivos copiados:

```text
2fd1bf9e6ec9d00f19a21b7a56c0fb5a1fa7048ec95fb614b18968df2271ce99  rtwtypes.h
adcffc6b11345da50251fbe5bde7f32e08558d6ce3a0cf664f14ebb521b03c78  STM32.c
b6c04339cd867a0c5c5c3a5594fc2d098b3ae4cb45f41be18fcdc57e2feba6a0  STM32_data.c
5165dcdc208a738c674341e560823822a26001e7b54a84593be3862fb9ce6869  STM32.h
9a1747123cc501672250c30e97e7b8124f6a96de8b24caa246a362e22fdfa4eb  STM32_private.h
762e2a9ee872dc183140246dc18251bf51d9736dc01d4f2b64277880de8a7c8e  STM32_types.h
```
