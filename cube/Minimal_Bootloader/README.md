# Project Description

This project implements a **minimal custom bootloader** for the **Nucleo-F091RC** (STM32F091RCTx). Flash is split into two regions: a small bootloader that runs after reset, and a separate application linked higher in memory. On startup the bootloader blinks the onboard LED; pressing the user button (**PC13**) triggers a handoff that copies the application’s vector table into SRAM, remaps memory, and jumps to the app’s reset handler.

The goal is to practice **memory partitioning**, **vector tables**, and **boot-time control transfer** without relying on ST’s built-in ROM bootloader or a full update protocol.

# Implementation Details

## Project layout

| Subproject | CubeIDE path | Linked at | Size (flash) |
|------------|--------------|-----------|--------------|
| Bootloader | `bootloader/` | `0x08000000` | 16 KB |
| Application | `application/` | `0x08004000` | Remaining flash (~240 KB) |

Each folder is its own STM32CubeIDE project with its own linker script (`STM32F091RCTX_FLASH.ld`).

## Memory map

```
0x08000000  ┌─────────────────────┐
            │  Bootloader (16K)   │  Blinks PA5 LED; button → jump
0x08004000  ├─────────────────────┤
            │  Application        │  Button interrupt toggles LED
            │  ...                │
0x08040000  └─────────────────────┘  (end of 256 KB flash)

SRAM (bootloader jump only):
0x20000000  Vector table copy (256 bytes) before SYSCFG remap
0x20000100  Application RAM (.data / .bss / stack) per app linker script
```

The application linker reserves the first **256 bytes** of SRAM (`ORIGIN = 0x20000100`) so the bootloader can place a vector table at `0x20000000` without overlapping the app’s runtime memory.

## Bootloader behavior (`bootloader/Core/Src/main.c`)

1. **GPIO** — **PA5** (LED) output; **PC13** (user button) input with pull-up.
2. **EXTI** — Falling edge on line 13 sets `jump_request`.
3. **Main loop** — Toggle LED with a busy-wait delay; when `jump_request` is set, call `jump_to_application()`.

### Jump sequence (`jump_to_application`)

When the user presses the button, the bootloader:

1. **Disables interrupts** (`__disable_irq()`).
2. **Copies 256 bytes** from `APP_BASE` (`0x08004000`) to `SRAM_VECTOR` (`0x20000000`) with `memcpy` — the app’s interrupt vector table.
3. **Sets MSP** from the first word at `APP_BASE` (initial stack pointer in the app image).
4. **Remaps SRAM to `0x00000000`** via `SYSCFG->CFGR1 |= 0x3` so the CPU fetches vectors from the copy in SRAM (required because the app is not linked at the default flash base).
5. **Branches** to the app’s `Reset_Handler` (second word at `APP_BASE + 4`)

## Application behavior (`application/Core/Src/main.c`)

- Linked at **`0x08004000`**; must be built and programmed separately from the bootloader.
- **PA5** — LED output (GPIO toggle, not PWM).
- **PC13** — EXTI13 falling edge toggles the LED in `EXTI4_15_IRQHandler`.
- Main loop is idle (`__NOP()`); all interaction is interrupt-driven.

## Build and flash

1. Open **`bootloader/`** in STM32CubeIDE, build, and flash to the Nucleo (programs the bottom 16 KB).
2. Open **`application/`**, build, and flash the application image (programs from `0x08004000` upward). Use “Download” / ST-Link; the debugger usually programs the correct offset when the linker script matches.
3. Reset the board: the **bootloader** runs first (blinking LED).
4. Press the **user button** to jump into the **application** (LED toggles on each press instead of bootloader blink).

If only one image is flashed, behavior will be wrong (e.g. app alone at `0x08004000` without the bootloader at reset, or bootloader without an app at `0x08004000`).

# Demonstration

[![Bootloader Demo](https://img.youtube.com/vi/oeL051KWvYE/hqdefault.jpg)](https://www.youtube.com/shorts/oeL051KWvYE)
