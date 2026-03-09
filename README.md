# GPS Garage Opener

A Flipper Zero external application that automatically replays a Sub-GHz signal
(e.g. a recorded garage-door remote) whenever you enter a configured GPS region.
Designed for cycling: mount the Flipper on your bike, ride home, and the garage
door opens on its own.

---

## Hardware

### Parts

| Item | Notes |
|------|-------|
| Flipper Zero | Any firmware ≥ 0.99 |
| uBlox GPS module | NEO-6M, NEO-7M, NEO-8M, M8N, etc. — any NMEA-capable module at 3.3 V |
| 4× Dupont wires | Female–female or female–male depending on module footprint |

### Wiring

```
uBlox module          Flipper Zero GPIO header
─────────────         ──────────────────────────
VCC  ──────────────►  pin 9   (3V3)
GND  ◄─────────────►  pin 11  (GND)
TX   ──────────────►  pin 14  (USART1 RX)
RX   ◄──────────────  pin 13  (USART1 TX)   ← optional, not used
```

The module must be set to 9600 baud / NMEA output (factory default for all
uBlox modules).

> **Note:** pin 13/14 are also used by the Flipper debug console. The app
> temporarily takes ownership of the UART while it is running. The debug
> console is restored when the app exits.

---

## Building

You need the [Flipper Zero firmware SDK](https://github.com/flipperdevices/flipperzero-firmware).

```bash
# Clone the firmware (if not already done)
git clone --recursive https://github.com/flipperdevices/flipperzero-firmware.git
cd flipperzero-firmware

# Copy this app into the applications_user directory
cp -r /path/to/FlipperuBloxSubGhz applications_user/gps_garage_opener

# Build the .fap
./fbt fap_gps_garage_opener

# Or flash directly to a connected Flipper
./fbt launch APPSRC=applications_user/gps_garage_opener
```

The compiled `.fap` will be at
`build/f7-firmware-D/.extapps/gps_garage_opener.fap`.
Copy it to `/ext/apps/GPIO/` on the Flipper's SD card.

---

## First-time setup

1. **Record your garage remote** using the Flipper's built-in Sub-GHz app
   (Sub-GHz → Read → press your remote → Save).
   The `.sub` file is saved to `/ext/subghz/` on the SD card.

2. **Drive / cycle to your garage entrance** (the spot where you want the door
   to open automatically).

3. Launch *GPS Garage Opener* from `Apps → GPIO`.

4. Wait for a GPS fix — the top-right corner shows `[Xsat]` when locked.

5. **Set the trigger point:** hold **UP** — the app saves your current GPS
   coordinates as the trigger location.

6. **Select the .sub file:** hold **DOWN** — navigate to your recorded file and
   press **OK**.

7. **Adjust radius** with **LEFT** / **RIGHT** (5 m steps).
   A 30–50 m radius works well for most driveways.

8. **Enable auto-tracking:** hold **OK** — status line shows `TRACKING`.

Config is saved automatically and persists across reboots
(`/ext/apps_data/gps_garage/config.txt`).

---

## Controls

| Button | Action |
|--------|--------|
| **OK** (short) | Transmit immediately |
| **OK** (long) | Toggle auto-tracking on / off |
| **UP** (long) | Capture current GPS position as trigger target |
| **DOWN** (long) | Browse SD card and select `.sub` file |
| **LEFT** (short) | Decrease trigger radius by 5 m |
| **RIGHT** (short) | Increase trigger radius by 5 m |
| **BACK** | Exit app |

---

## Config file format

`/ext/apps_data/gps_garage/config.txt` — plain text, one key=value per line:

```
lat=47.12345678
lon=8.23456789
radius=50.0
cooldown=30
file=/ext/subghz/garage.sub
tracking=1
```

You can edit this file directly from the Flipper file manager or from a PC.

---

## How it works

1. A background thread reads NMEA sentences from the GPS module via USART1
   at 9600 baud.
2. `$GPRMC` sentences are parsed for position; `$GPGGA` for satellite count.
3. A 1-second timer checks if the current position is within `radius` metres
   of the target using the Haversine formula.
4. If in range and the cooldown period has elapsed, a separate thread loads
   the `.sub` file, configures the CC1101 radio, and transmits.
5. The cooldown (default 30 s) prevents continuous re-triggering while you are
   near the garage.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| `[NoFix]` after several minutes outdoors | GPS antenna blocked — most breakout boards need open sky |
| Door does not open automatically | Tracking not enabled (hold OK), or no .sub file selected |
| "TRANSMITTING..." hangs | .sub file corrupt or unsupported protocol |
| Region error (TX fails silently) | Frequency not permitted in your region; re-record at the correct frequency |
| Wrong coordinates captured | Wait for more satellites before pressing hold-UP |
