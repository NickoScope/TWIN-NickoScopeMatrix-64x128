# TWIN-NickoScopeMatrix-64x128: a virtual twin of an LED panel

This fork of [esp32sim](https://github.com/joakimeriksson/esp32sim) by Joakim Eriksson runs the
unmodified firmware of [AnimatedPixelClock](https://github.com/NickoScope/AnimatedPixelClock) for a
Waveshare ESP32-S3-RGB-Matrix board driving a 128x64 HUB75 panel. Mac users see on screen what the
LEDs would show, and can operate the IR remote and the knob. The fork's work is on the branch
`nickoscope/twin`. `main` follows upstream.

## What the branch adds

- **Octal flash and storage**
  - The module's octal (OPI) flash: `--flash-id`, and 4-byte and Macronix octal command bytes.
  - `--flash-persist FILE`: the flash chip as a file, written through.
- **Display**
  - LCD_CAM in i8080 mode, streaming its GDMA ring to the board at the programmed PCLK.
  - Transactions end with TRANS_DONE.
  - The `hub75` crate: a generic HUB75 decoder (shift, latch, OE-gated light per LED per refresh) and a renderer with LED dots and optional GOB glow.
  - `--board hub75-panel`: the panel, its SHTC3 on I2C0, and its inputs as pin-level devices: an NEC IR receiver on GPIO0 (wired-AND with the knob's switch and BOOT) and an EC11 knob.
  - `web/panel.html`: the panel as light, a remote, a knob, a camera-shutter view of the 1/32 scan.
- **Peripherals and reset**
  - An SD/MMC host with an empty slot.
  - The radio's IQ calibration completing without an AP.
  - A chip reset that keeps the virtual AP and network.
- **Network and flashing**
  - `--hostfwd tcp|udp:HOST-GUEST`: inbound forwarding on 127.0.0.1.
  - `--net bridge:PATH`: the guest on the real LAN through socket_vmnet (vmnet bridged).
  - A lossless USB-Serial/JTAG channel (`/usj`), DTR/RTS resets per the ESP32-S3 TRM, and RFC 2217 (`--serial-tcp`). The ESP Web Tools web flasher and esptool flash the guest as they would a chip.
- **Timing**
  - `--cpi N.N`, a fractional uniform cycles per instruction, calibrated against the panel.
  - Real-time pacing catches a lag up (at most 1.5× real time) instead of dropping any lag over 0.5 s, so the firmware's clock of day stays with the world between its NTP syncs; only time the engine did not run (a stopped process), a lag beyond 30 s or a host sleep is given up, and logged.

How to run the twin, and the design with its sources: `tools/twin/` in AnimatedPixelClock.

Upstream's license (MIT) applies to everything here.
