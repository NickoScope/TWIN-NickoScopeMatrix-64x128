#pragma once
// Host shim for FW src/ir/ir.h: only the seam control.cpp calls, backed by the real ir_map.h
// Decoder in oracle.cpp. IR_PIN as in FW ir.h:41-43.
#include <stdint.h>
#include "ir_map.h"
#ifndef IR_PIN
#define IR_PIN 0
#endif
int8_t irTakeRotate();
bool   irOkDown(uint32_t nowMs);
