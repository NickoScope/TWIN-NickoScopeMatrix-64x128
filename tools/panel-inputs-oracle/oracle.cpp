// Host oracle for the twin's panel inputs: feed the GPIO0/IO45/IO46 edges esp32sim's hub75-panel
// board produced (a --vcd file) through the REAL firmware-side code, built on the host:
//   - IRremoteESP8266 2.9.0's capture, transcribed from IRrecv.cpp gpio_intr() (194-231) and
//     read_timeout() (the ISR is compiled out under UNIT_TEST), 15 ms timeout (FW ir.cpp:69),
//     then the REAL IRrecv::decode() (UNIT_TEST build, setUnknownThreshold(12) as FW ir.cpp:371);
//   - the REAL ir::Decoder from FW src/ir/ir_map.h, with the owner's ten codes bound as learned;
//   - the REAL FW src/control/control.cpp (IR_ENABLED + IR_RX_ENABLED, so the 40 ms clamp and the
//     seam are in), sampled once per simulated millisecond through its own controlLoop() fallback.
// Usage: oracle <file.vcd> [phase_us=500] [loop_latency_ms=2]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Arduino.h"   // FW tools/control shim: g_pin, g_ms, Serial
#include "control.h"
#include "IRrecv.h"
#include "IRutils.h"
#include "ir_map.h"

int g_pin[64];
uint32_t g_ms = 0;
SerialShim Serial;

static ir::Decoder s_dec;
int8_t irTakeRotate() { return s_dec.takeRotate(); }
bool irOkDown(uint32_t nowMs) { return s_dec.okDown(nowMs); }

struct Edge { uint64_t ps; int pin; int level; };

static std::vector<Edge> readVcd(const char *path) {
  std::ifstream f(path);
  std::map<std::string, int> id;   // "s0" -> 0
  std::vector<Edge> out;
  std::string line;
  uint64_t now = 0;
  bool defs = true;
  while (std::getline(f, line)) {
    if (defs) {
      if (line.rfind("$var", 0) == 0) {
        std::istringstream is(line); std::string v, w, one, code, name; is >> v >> w >> one >> code >> name;
        if (name.rfind("gpio", 0) == 0) id[code] = atoi(name.c_str() + 4);
      }
      if (line.rfind("$enddefinitions", 0) == 0) defs = false;
      continue;
    }
    if (line.empty()) continue;
    if (line[0] == '#') { now = strtoull(line.c_str() + 1, nullptr, 10); continue; }
    if (line[0] == '0' || line[0] == '1') {
      auto it = id.find(line.substr(1));
      if (it != id.end()) out.push_back({now, it->second, line[0] - '0'});
    }
  }
  return out;
}

// ---- IRrecv capture, transcribed (IRrecv.cpp:194-231, read_timeout, resume)
static const uint16_t kBuf = 256;         // FW ir.cpp:69
static const uint32_t kTimeoutUs = 15000;  // FW ir.cpp:69 (15 ms)
struct Capture {
  uint16_t raw[kBuf + 1]; uint16_t rawlen = 0; bool overflow = false;
  uint8_t state = kIdleState; uint32_t start = 0; uint64_t timeoutAt = 0; bool armed = false;
  void resume() { state = kIdleState; rawlen = 0; overflow = false; armed = false; }
  void edge(uint32_t now) {
    uint16_t rl = rawlen;
    if (rl >= kBuf) { overflow = true; state = kStopState; }
    if (state == kStopState) return;
    if (state == kIdleState) { state = kMarkState; raw[rl] = 1; }
    else raw[rl] = (uint16_t)((now < start ? (UINT32_MAX - start + now) : (now - start)) / kRawTick);
    rawlen = rawlen + 1;
    start = now;
    timeoutAt = now + (uint64_t)kTimeoutUs; armed = true;   // timerWrite(0) + alarm enable
  }
};

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: oracle file.vcd [phase_us] [loop_latency_ms]\n"); return 2; }
  std::vector<Edge> ev = readVcd(argv[1]);
  const uint64_t phaseUs = argc > 2 ? strtoull(argv[2], nullptr, 10) : 500;
  const uint64_t loopMs = argc > 3 ? strtoull(argv[3], nullptr, 10) : 2;

  // The owner's map as the live panel had it (research vp_study_io.json; FW ir_map.h names).
  const uint64_t codes[10] = {0xFF708F, 0xFF58A7, 0xFFE01F, 0xFF41BE, 0xFF28D7, 0xFFC03F, 0xFF19E6, 0xFF12ED, 0xFF40BF, 0xFFC936};
  const uint8_t fns[10] = {ir::kFnCcw, ir::kFnCw, ir::kFnOk, ir::kFnLong, ir::kFnPower, ir::kFnBrightUp, ir::kFnCarousel, ir::kFnHome, ir::kFnBrightDown, ir::kFnMediaToggle};
  s_dec.reset();
  for (int i = 0; i < 10; i++) { s_dec.map().setFn(i, fns[i], 0); s_dec.map().bind(i, (uint8_t)decode_type_t::NEC, codes[i]); }

  IRrecv rx(0, kBuf, 15, false);
  rx.setUnknownThreshold(12);   // FW ir.cpp:371
  Capture cap;
  bool decodePending = false; uint64_t decodeAtUs = 0;

  // Idle levels (the board's input_levels): GPIO0 high, IO45/IO46 low.
  g_pin[0] = 1; g_pin[45] = 0; g_pin[46] = 0;
  g_ms = 1;
  controlBegin();   // the esp_timer shim fails: controlLoop() samples, the firmware's own fallback

  size_t i = 0;
  const uint64_t endUs = ev.empty() ? 0 : ev.back().ps / 1000000 + 3000000;
  int nFrames = 0;
  for (uint64_t ms = 2; ms * 1000 + phaseUs <= endUs; ms++) {
    const uint64_t tUs = ms * 1000 + phaseUs;
    // Everything up to this sample, in time order: edges (the ISR), the timeout, loop()'s decode.
    for (;;) {
      uint64_t nextEdge = i < ev.size() ? ev[i].ps / 1000000 : UINT64_MAX;   // micros()
      uint64_t nextTimeout = cap.armed ? cap.timeoutAt : UINT64_MAX;
      uint64_t nextDecode = decodePending ? decodeAtUs : UINT64_MAX;
      uint64_t n = std::min(nextEdge, std::min(nextTimeout, nextDecode));
      if (n > tUs) break;
      if (n == nextTimeout) {   // read_timeout(): stop if anything was captured
        cap.armed = false;
        if (cap.rawlen) { cap.state = kStopState; decodePending = true; decodeAtUs = n + loopMs * 1000; }
        continue;
      }
      if (n == nextDecode) {    // irLoop(): FW ir.cpp:393-403
        decodePending = false;
        decode_results r;
        static uint16_t buf[kBuf + 1];
        memcpy(buf, cap.raw, sizeof(buf));
        if (!cap.overflow) buf[cap.rawlen] = 0;
        r.rawbuf = buf; r.rawlen = cap.rawlen; r.overflow = cap.overflow;
        cap.resume();
        const uint32_t now = (uint32_t)(n / 1000);
        if (r.overflow) { printf("%7u ms  overflow\n", now); continue; }
        if (rx.decode(&r)) {
          ir::Frame f; f.proto = (uint8_t)r.decode_type; f.value = r.value; f.repeat = r.repeat; f.unknown = r.decode_type == decode_type_t::UNKNOWN;
          ir::Outcome o = s_dec.frame(now, f);
          nFrames++;
          printf("%7u ms  frame %-7s 0x%06llX%s  rawlen %u -> outcome %d slot %d fn %s\n", now, typeToString(r.decode_type).c_str(),
                 (unsigned long long)(r.repeat ? 0 : r.value), r.repeat ? " (repeat)" : "", r.rawlen, o.kind, o.slot + 1, ir::fnInfo(o.fn).name);
        } else {
          printf("%7u ms  capture of %u entries not decoded\n", now, r.rawlen);
        }
        continue;
      }
      const Edge &e = ev[i++];
      if (e.pin == 0 || e.pin == 45 || e.pin == 46) {
        g_pin[e.pin] = e.level;
        if (e.pin == 0) cap.edge((uint32_t)(e.ps / 1000000));   // attachInterrupt(0, gpio_intr, CHANGE)
      }
    }
    g_ms = (uint32_t)ms;   // millis() at this sample
    controlLoop();
    for (CtrlEvent c = controlTake(); c != CTRL_NONE; c = controlTake())
      printf("%7u ms  EVENT %s\n", g_ms, c == CTRL_CW ? "CW" : c == CTRL_CCW ? "CCW" : c == CTRL_PRESS ? "PRESS" : "LONG");
  }
  CtrlStats st; controlStats(&st);
  printf("frames %d, ignored %u | control: cw %u ccw %u press %u long %u detent %d\n", nFrames, s_dec.ignored(), st.cw, st.ccw, st.press, st.longPress, st.detent);
  return 0;
}
