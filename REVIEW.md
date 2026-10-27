# Code review — ESP32-C6 + HLK-LD2412 presence display

Status of the original review against `DOC/HLK-LD2412 Serial Communication Protocol.pdf`
(v1.01) and the ESP32-C6 datasheet.

## Fixed (PRs #1-#7 and the cleanup pass)

All section-A blocking bugs and section-B real-time issues are resolved:

- Report frames are decoded into `ld2412_data_t` and consumed by `main` — no more
  uninitialised `targets[]` reads (was A1, A6, A7).
- `send_command()` wraps commands in the correct `FD FC FB FA … 04 03 02 01`
  framing, and every config command waits for its ACK (was A2, B4).
- The unsigned-underflow OOB in the ACK loops is gone; `wait_ack()` scans a
  bounded buffer with `size_t` indexes (was A3).
- The radar UART is on GPIO18/19, off the U0TXD/U0RXD console pads; a warning
  still fires if the pins are configured back onto 16/17 (was A4).
- Firmware version is parsed at the correct offsets and printed as hex
  (`V1.10.24041810` format), searching for the ACK header first (was A5).
- The main loop drains the ~10 Hz radar stream continuously and only throttles
  the LCD refresh; `lv_timer_handler()` runs every 10 ms (was B1, B2).
- `ld2412_read_data()` keeps a persistent accumulation buffer with header
  resync, so frames split across reads are no longer dropped (was B3).
- The made-up "start stream" command is gone; the radar streams on its own
  (was B5).
- `ld2412_init()` propagates `esp_err_t` from the UART calls (was B6);
  `take_frame()` validates lengths before dereferencing (was B7).
- Engineering-mode frames (Table 13: max gates + 14+14 gate energies) are
  decoded by the parser.
- `dependencies.lock` and `build/` are gitignored.

## Cleanup pass

- Removed the vendor-template leftovers the project does not use: `SD_Card/`
  (the board has no SD slot — it came from the Waveshare example),
  `Wireless/` (Bluetooth was never initialised), `LVGL_Example` demo,
  `main.c_org`, `RGB_Example`, and the unused `font_text_18` source
  (`font_digits_85` is now the distance digit font).
- `Kconfig.projbuild` no longer redefines ESP-IDF/LVGL symbols
  (`BT_BLE_*_FEATURES_SUPPORTED`, `LV_*`), which produced Kconfig collision
  warnings and enabled the LVGL demos.
- `sdkconfig` is untracked and regenerated from `sdkconfig.defaults`, which now
  pins `CONFIG_IDF_TARGET="esp32c6"` and drops the inherited ESP32-S3 Bluetooth
  settings.
- `CONFIG_LD2412_ENGINEERING_MODE` enables engineering mode at boot so the
  display shows per-gate energy bars and the 1 Hz log prints all 14+14 gate
  energies.
- The onboard RGB LED mirrors the connection state (red = no frames,
  green = receiving).
- Radar gate configuration commands are implemented:
  `ld2412_get/set_resolution` (0x0001/0x0011, applies after radar reboot),
  `ld2412_get/set_basic_params` (0x0002/0x0012: min/max gate, unoccupied
  duration, OUT polarity), and per-gate `ld2412_get/set_motion_sensitivity`
  (0x0003/0x0013) and `ld2412_get/set_static_sensitivity` (0x0004/0x0014).
  Sensitivities persist on the radar; boot only *reads and logs* the current
  values — nothing is written unless the set functions are called.

## Still open / possible follow-ups

- No automated tests: verifying real behaviour needs the radar + LCD hardware
  (boot probe, firmware-version read, live distances, gate bars, no-target
  timeout, sensitivity tuning).
- `ld2412_set_*` functions exist but no menuconfig/NVS UI applies them; tuning
  today means a code change or the vendor's app.
- Further protocol commands exist but are unimplemented (dynamic background
  correction 0x000B/0x001B, restart 0x00A3, factory reset 0x00A2, light-sense
  auxiliary control 0x000C/0x001C, radar Bluetooth toggle 0x00A4, MAC 0x00A5) —
  add them if needed.
