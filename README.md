# Rockwolf 32 ESC — published source and firmware manifest

This repository publishes the **source** and the firmware **manifest** for the Rockwolf 32 ESC
(an AM32-derived firmware fork for the FD6288 / STM32F051 board).

**The browser tools were retired from this site on 2026-09-17.** The Rockwolf tuner
(`index.html`) and the ESC diagnostic page (`diag/index.html`) are no longer served here, and
nothing in this repository links to them. Their canonical sources remain in the private firmware
repository as `RockwolfTuner.html` and `RockwolfDiag.html`.

## What is here

| path | what |
|---|---|
| `src/` | complete corresponding source for the published firmware builds (GPLv3) |
| `firmware/manifest.json` | build manifest; empty until the Rockwolf 2.0 release ships |
| `LICENSE` | GPLv3 |
| `.nojekyll` | Pages passthrough |

## Source and licence

The firmware is an **AM32 derivative** and is distributed under the **GNU General Public
License v3** — see [LICENSE](LICENSE). The complete corresponding source for the published
builds is in **[src/](src/)**; `src/Inc/targets.h` carries the version tag (`RW_FW_TAG`) that
identifies which build a given tree produces, and the build command is:

```sh
make ARM_SDK_PREFIX=arm-none-eabi- FD6288_F051
```

Upstream AM32 lives at am32.ca. This project is not affiliated with it.
