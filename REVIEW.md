# Code review — ESP32-C6 + HLK-LD2412 presence test

Reviewed against `DOC/HLK-LD2412 Serial Communication Protocol.pdf` (v1.01) and the
ESP32-C6 datasheet. Ordered roughly by severity.

## A. Blocking bugs

### A1. `main.c` displays uninitialised stack memory
`ld2412_read_frame()` fills `frame->buf`, `start`, `endData` and sets `target_count = 0`,
but it **never writes `frame->targets[]`**. `ld2412_parse_basic()` decodes into a *local*
`ld2412_basic_target_t` and only logs it — the result is thrown away.
So in `main.c`:

```c
uint16_t distance = frame.targets[0].distance_mm;   // never written by the driver
uint8_t  signal   = frame.targets[0].signal_strength;
```

reads whatever was on the stack. The number on the LCD is unrelated to the sensor; the only
real data is what `ESP_LOGI` prints inside `ld2412_parse_basic()`.

Fix: give the parser an out-parameter (`ld2412_parse_basic(buf, start, end, &out)`) and have
`ld2412_read_frame()`/`ld2412_parse_frame()` fill a decoded result struct that `main` reads.

### A2. `send_command()` uses the *data* frame header, not the *command* header
```c
static const uint8_t FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};  // report frames
static const uint8_t FRAME_TAIL[]   = {0xF8, 0xF7, 0xF6, 0xF5};
```
Per §2.1 of the protocol, command frames must be `FD FC FB FA … 04 03 02 01`.
`send_command()` wraps commands in `F4 F3 F2 F1 … F8 F7 F6 F5`, so **every command that goes
through it is ignored by the radar**: `ld2412_enable_config()`, `ld2412_end_config()` and
`ld2412_read_firmware_version()`. That is why the firmware-version block in `main.c` had to be
commented out. The hand-rolled `ld2412_enable_configuration()` / `ld2412_set_output_mode()` /
`ld2412_exit_configuration()` use the correct header, which is why those work — the two
families should be merged onto one correct `send_command()`.

### A3. Unsigned underflow → out-of-bounds read in all three ACK loops
```c
for (int i = 0; i <= len - sizeof(ack); i++)
```
`sizeof` is `size_t`, so `len - sizeof(ack)` is **unsigned**. On a timeout (`len == 0`, which
happens whenever the radar is slow or unplugged) this is ~4.29e9, the comparison promotes `i`
to unsigned, and the loop walks off the end of the 1023-byte stack buffer until it faults.
Present in `ld2412_set_output_mode`, `ld2412_enable_configuration`, `ld2412_exit_configuration`.

Fix: `for (int i = 0; i + (int)sizeof(ack) <= len; i++)`.

### A4. UART1 is mapped onto the console pins
```c
#define LD2412_TX_PIN 16
#define LD2412_RX_PIN 17
```
On ESP32-C6, GPIO16 = U0TXD and GPIO17 = U0RXD (datasheet §2, IO MUX), and the project builds
with `CONFIG_ESP_CONSOLE_UART_NUM=0`. UART1 is routed through the GPIO matrix onto the same
pads the ROM/console UART drives, so boot log and `ESP_LOGx` output are injected into the
radar's RX line and the radar's replies collide with the console. Pick free pins on the
1.47" board header (e.g. GPIO18/19/20/23) and, if you keep UART0 on 16/17, leave them alone.
Also note the naming: `LD2412_TX_PIN` is passed as `uart_set_pin`'s *tx* argument, so it is the
ESP32's TX → the radar's RX.

### A5. Firmware-version response is parsed at the wrong offsets and formatted as decimal
Protocol example ACK:
`FD FC FB FA | 0B 00 | A0 01 | 00 00 | 12 24 | 10 01 | 10 18 04 24 | 04 03 02 01`
→ status at byte 8-9, firmware type at 10-11, major at 12-13, minor at 14-17, and all fields
are read as **hex** digits (`V1.10.24041810`).

The code reads type at 8, major at 10, minor at 12 (all off by two), assumes the ACK starts at
byte 0 of the buffer (a queued target frame in front of it breaks that), and prints with `%d` /
`%08lu`, so even with correct offsets it would print `V272.0.…` instead of `V1.10.24041810`.
Use `%X`/`%02X` and search for the `FD FC FB FA` header first.

### A6. `frame.targets[0].signal_strength = -1` is not a valid sentinel
`signal_strength` is `uint8_t`, so `-1` becomes `255`, which passes `if (signal >= 20)`.
The "did the driver actually write this?" check never triggers. Use the parser's return value.

### A7. `ld2412_parse_frame()` is called after the data is consumed
`main.c` reads `frame.targets[0]` on lines 77-78 and only calls `ld2412_parse_frame(&frame)`
on line 93. Even once A1 is fixed, the order has to be read → parse → use.

## B. Real-time / protocol behaviour

### B1. 2-second loop vs. a ~10 Hz radar stream
The radar emits a basic frame (21 bytes) roughly every 100 ms. The loop reads at most 64 bytes
every 2 s, so the 1024-byte driver RX buffer overflows within ~5 s and `uart_read_bytes()`
always returns the *oldest* bytes: the display lags reality by seconds and never catches up.
Read continuously (block on `uart_read_bytes` with a short timeout) and throttle only the
LCD refresh.

### B2. `lv_timer_handler()` every 2 s
The Waveshare template (`main.c_org`) calls it every 10 ms. At 2 s LVGL never runs its refresh
timer on time; expect frozen/torn rendering and multi-second latency. Keep the 10 ms LVGL loop
and update the labels from the sensor data, or move the radar into its own FreeRTOS task and
publish samples through a queue/mutex-protected struct (LVGL is not thread-safe — if you split
the tasks you must add a mutex, since this project has no `esp_lvgl_port` lock).

### B3. `ld2412_read_frame()` has no resynchronisation across reads
Each call reads into a fresh 64-byte buffer and gives up if the frame is split across the read
boundary ("Incomplete frame", "No valid frame found"). With back-to-back reads a fixed fraction
of frames is always lost. Keep a persistent accumulation buffer (or use
`uart_enable_pattern_det_baud_intr` / a dedicated RX task with an event queue) and consume
whole frames out of it.

### B4. Command sequencing does not honour the ACK contract
`main.c` fires enable-config → set-mode → exit-config back to back. Each command *must* be
followed by its ACK (§2.4.1) before the next is sent; the current ACK reads also swallow any
target frames that arrive in between, and a failed ACK is only logged, never acted on.
Also: engineering mode is **off after power-on** (§2.2.8), so the `set_output_mode(…, true)`
round-trip in `main.c` is a no-op in the default configuration.

### B5. `ld2412_start_stream()` sends a made-up command
`{0xAA, 0x00, 0x01, 0x01}` is not in the protocol; the radar streams automatically. Delete it.

### B6. `ld2412_init()` ignores every error
`uart_param_config`, `uart_set_pin`, `uart_driver_install` all return `esp_err_t` and are
dropped; the function unconditionally returns `ESP_OK`. Same for `LCD_Init()`/`SD_Init()` calls
in `main`. Use `ESP_RETURN_ON_ERROR`/`ESP_ERROR_CHECK`.

### B7. `ld2412_parse_frame()` dereferences `buf[end - 1]` before validating `end`
A short/corrupt frame gives `end <= 0` and reads out of bounds.

## C. Smaller issues

- `ld2412_set_output_mode()` comment says "Payload length = 5" while the frame says `02 00`
  (correct value, wrong comment); the `bool basic_mode` argument would read better as an enum
  (`LD2412_MODE_BASIC` / `LD2412_MODE_ENGINEERING`), since `0x63` = *close* engineering mode.
- Three 1023-byte ACK buffers on the stack; `ld2412_enable_configuration()` also zeroes it with
  a hand-written loop instead of `memset`. 64 bytes is plenty for any ACK.
- `decode_status()` returns a pointer to a `static` buffer — not reentrant, and the returned
  string embeds a `\n` that then goes through `ESP_LOGI`.
- `main.c`: labels are `lv_obj_set_width(…, 200)` on a 172x320 panel — wider than the screen.
  `lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP)` plus a width ≤172 is what the
  "prevent shifting" comment wants.
- `main.c`: magic `20` energy threshold — name it, and prefer the frame's own target-state byte
  (0x00 no target / 0x01 moving / 0x02 static / 0x03 both) over a hand-rolled threshold.
- `main.c`: on target loss the labels keep the last value forever; add a "no target"/timeout state.
- `main.c`: `"Signal: %3d/%3d", last_signal, signal` prints the sentinel garbage as the second
  number; `esp_mac.h` is included but unused; `freertos/FreeRTOS.h` and `freertos/task.h` are
  used but only included transitively.
- `LD2412.h` has `#pragma once` twice; `ld2412_target_t.distance_mm` is misnamed — the protocol
  reports **cm** (§2.3.2 Table 11), which is also why `main.c`'s `distance / 100.0f` happens to
  give metres.
- The engineering-mode parser is a stub; Table 13 (max moving/static gate + 14+14 gate energies
  + light + OUT pin) is straightforward to add and is the interesting data for tuning
  sensitivity per gate.
- Per-frame `ESP_LOGI` at 10 Hz will dominate the console; drop to `ESP_LOGD`.
- Repo hygiene: `build/` (25 MB) and `sdkconfig` are committed — both should be in
  `.gitignore` (`sdkconfig` is regenerated from `sdkconfig.defaults`).
- `sdkconfig.defaults` is inherited from an ESP32-S3 board template: `CONFIG_SPIRAM*`
  (the C6 has no PSRAM), `CONFIG_BT_CLASSIC_ENABLED` / `A2DP` / `SPP` (C6 is BLE-only) and
  `CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST` are silently ignored or misleading. Bluetooth is
  enabled at all while `Wireless_Init()` is commented out — disabling it frees a lot of flash.
- `dependencies.lock` pins `lvgl` and `led_strip` to absolute local paths
  (`/home/honi/Documents/...`), which will not resolve on another machine.
- `main.c_org` is a leftover copy of the vendor example — it is not built (not in
  `CMakeLists.txt`) but is confusing; keep it out of `main/` or delete it.
