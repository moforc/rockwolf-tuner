# Rockwolf Tuner

Browser-based configuration tool for the **Rockwolf 32 ESC** (an AM32-derived firmware fork
for the FD6288 / STM32F051 board). No installer, no companion app: the page talks to the ESC
directly through a USB-serial programmer using the browser's Web Serial API.

## Requirements

- **Chrome or Edge on a desktop** (Chrome on Android also works with a USB-C programmer).
  Firefox and Safari do not implement Web Serial.
- **A secure context.** Web Serial is only available over `https://` or on `http://localhost`.
  A plain `http://` address, including a LAN IP, cannot reach the programmer.
- **A USB-serial programmer** (CH340, CP2102 or FTDI) on the ESC's signal lead, and a battery
  on the ESC. The programmer powers its own side; the ESC needs the pack.

## Using it

1. Open this page, plug the programmer into the ESC's signal lead, power the ESC.
2. Press **Connect** and pick the programmer if the browser asks.
3. Read the settings, change what you need, and **Write Settings**.

If settings on your ESC are not writable, the tool greys those fields and says why rather than
letting you write a value the firmware cannot use.

## Firmware downloads

Downloads open with the **Rockwolf 2.0** release: one image that runs brushed *and* brushless
motors, with the mode selected in the tool, alongside the factory brushless and brushed images
for anyone who wants to go back. They are published together with their source, as the licence
below requires.

## Source and licence

The firmware is an **AM32 derivative** and is distributed under the **GNU General Public
License v3** — see [LICENSE](LICENSE). The complete corresponding source for the published
builds is in **[src/](src/)**; `src/Inc/targets.h` carries the version tag
(`RW_FW_TAG`) that identifies which build a given tree produces, and the build command is:

```sh
make ARM_SDK_PREFIX=arm-none-eabi- FD6288_F051
```

Upstream AM32 lives at am32.ca. This project is not affiliated with it.

## Repository layout

| path | what |
|---|---|
| `index.html` | the tuner — one self-contained page, served by GitHub Pages |
| `firmware/` | published `.hex` builds plus `manifest.json`, which the page reads |
| `src/` | firmware source (GPLv3) |
| `LICENSE` | GPLv3 |
| `README.md` | this file |

**`index.html` is generated.** It is built from the canonical tuner by
`deploy-tuner-site.sh` in the firmware repository, which strips the tool's harness-only
endpoints and points it at `firmware/manifest.json` instead. Edit the canonical tuner, not this
file — the next build overwrites it.
