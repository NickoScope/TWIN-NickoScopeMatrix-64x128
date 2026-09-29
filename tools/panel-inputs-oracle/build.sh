#!/bin/sh
# Host oracle for the hub75-panel inputs (esp32s3/src/board/panel_inputs.rs): replays the GPIO0 /
# IO45 / IO46 edges of an esp32sim run (--vcd) through the firmware's own sources built on the
# host: IRremoteESP8266 2.9.0's decode() (UNIT_TEST build, the capture ISR transcribed in
# oracle.cpp), the real src/ir/ir_map.h Decoder with the owner's ten codes bound, and the real
# src/control/control.cpp with the IR seam. It checks the waveforms, not the emulator.
#
#   FW=~/AnimatedPixelClock-twin \
#   IRR=~/AnimatedPixelClock-netbroker/.pio/libdeps/matrix-waveshare-rgb/IRremoteESP8266 \
#   tools/panel-inputs-oracle/build.sh /tmp/pio-oracle
#   esp32sim --board hub75-panel ... --script tools/panel-inputs-oracle/scenario.txt --vcd run.vcd
#   /tmp/pio-oracle/oracle run.vcd [sampler_phase_us=500] [loop_latency_ms=2]
set -e
: "${FW:?set FW to the AnimatedPixelClock source tree}"
: "${IRR:?set IRR to the IRremoteESP8266 2.9.0 the firmware builds with}"
OUT=${1:-/tmp/panel-inputs-oracle}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT/obj" "$OUT/src/control" "$OUT/src/ir"
# control.cpp includes "../ir/ir.h": give it the seam only, next to symlinks of the real files.
ln -sf "$FW/src/control/control.cpp" "$OUT/src/control/control.cpp"
ln -sf "$FW/src/control/control.h" "$OUT/src/control/control.h"
ln -sf "$FW/src/ir/ir_map.h" "$OUT/src/ir/ir_map.h"
cp "$HERE/ir_seam.h" "$OUT/src/ir/ir.h"
for f in "$IRR"/src/*.cpp; do
  c++ -std=c++17 -O1 -w -DUNIT_TEST -I "$IRR/src" -I "$IRR/test" -c "$f" -o "$OUT/obj/$(basename "$f" .cpp).o" &
done
wait
DEFS="-DCONTROL_ENCODER_ENABLED -DBOARD_WAVESHARE_RGB_MATRIX -DIR_ENABLED -DIR_RX_ENABLED"
INC="-I $FW/tools/control -I $OUT/src/control -I $OUT/src/ir"
c++ -std=c++17 -O1 -Wall -Wno-unused-function $DEFS $INC -c "$OUT/src/control/control.cpp" -o "$OUT/control.o"
c++ -std=c++17 -O1 -Wall -Wno-unused-function $DEFS -DUNIT_TEST $INC -I "$IRR/src" -I "$IRR/test" -c "$HERE/oracle.cpp" -o "$OUT/oracle.o"
c++ "$OUT/oracle.o" "$OUT/control.o" "$OUT"/obj/*.o -o "$OUT/oracle"
echo "built $OUT/oracle"
