#!/bin/sh
# Testes host-only dos módulos puros da bancada, com ASan/UBSan.
# Não são qualificação física: não tocam HAL, FreeRTOS, USB nem motores.
set -e
cd "$(dirname "$0")/.."

PURE="App/Src/mtf_parser.c App/Src/actuators/motor_arbiter.c App/Src/dshot_codec.c
      App/Src/actuators/esc_kiss.c App/Src/link/bench_frame.c App/Src/link/bench_proto.c
      App/Src/power_cal.c App/Src/attitude.c App/Src/control/lqg_meas.c
      App/Src/control/lqg_ctrl.c App/Src/control/ekf3.c App/Src/control/quat.c App/Src/control/lqg_sim.c
      App/Src/control/lqg_model/STM32.c
      App/Src/control/lqg_model/STM32_data.c"
CFLAGS="-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined
         -fno-omit-frame-pointer -g -DCTRL_HOST_BUILD -IApp/Inc -Itests/stubs -Itests"
OUT="${TMPDIR:-/tmp}/aeroroute-bench-tests"
mkdir -p "$OUT"
failures=0

run_suite() {
  name=$1
  src=$2
  extra=$3
  if gcc $CFLAGS "$src" $extra $PURE -lm -o "$OUT/$name" 2>"$OUT/$name.log" &&
     "$OUT/$name" >"$OUT/$name.out" 2>&1; then
    echo "PASS  $name"
  else
    echo "FAIL  $name"
    cat "$OUT/$name.log" "$OUT/$name.out" 2>/dev/null || true
    failures=$((failures + 1))
  fi
}

for src in tests/test_*.c; do
  case "$src" in
  tests/test_flight_command.c)
    # Executor do DroneLink: o teste fornece control_task, motores e bateria.
    extra="App/Src/dronelink/flight_command.c tests/support/fake_control.c" ;;
  tests/test_dronelink_protocol.c)
    # Pilha DroneLink com MAVLink real; o transporte é trocado por uma captura.
    extra="App/Src/dronelink/dronelink_protocol.c App/Src/dronelink/dronelink_transaction.c
           App/Src/dronelink/dronelink_telemetry.c App/Src/dronelink/flight_command.c
           tests/support/fake_control.c -IMiddlewares/Third_Party/mavlink
           -fno-sanitize=alignment" ;;
  *) extra="" ;;
  esac
  run_suite "$(basename "$src" .c)" "$src" "$extra"
done

if [ "$failures" -eq 0 ]; then
  echo
  echo "all bench suites passed"
else
  echo "$failures suite(s) failed"
  exit 1
fi
