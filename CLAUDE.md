# GPS Garage Opener (Flipper Zero external app)

Flipper Zero FAP: uBlox GPS module + Sub-GHz replay to open a garage door automatically when nearby.

## Build setup

- This repo: `~/dev/FlipperuBloxSubGhz`
- Firmware repo: `~/dev/flipperzero-firmware` (official firmware)
- This repo is symlinked into the firmware tree:
  `~/dev/flipperzero-firmware/applications_user/gps_garage_opener -> ~/dev/FlipperuBloxSubGhz/`
- Do not copy files into the firmware tree; edit here, build there.

## Commands (run from `~/dev/flipperzero-firmware`)

- Build: `./fbt fap_gps_garage_opener`
  - Output: `build/f7-firmware-DC/.extapps/gps_garage_opener.fap` (the folder name changes with the build config)
- Build + upload + launch on a connected Flipper: `./fbt launch APPSRC=applications_user/gps_garage_opener`
- `appid` in `application.fam` is `gps_garage_opener`
