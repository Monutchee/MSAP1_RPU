# MSAP1 RPU repository guidance

## Purpose and routing

- This repository contains the FreeRTOS applications for ZynqMP R5 cores 0
  and 1 plus their shared OpenAMP and board-control code.
- Read `docs/AD7771.md` before changing the ADC path. Treat
  `common/include/ad7771.hpp` as the public ADC abstraction; application code
  should not duplicate AD7771 register or AXI register accesses.
- R5 core 0 owns the AD7771, the status LED heartbeat, and the APU control
  endpoint. R5 core 1 has no ADC or KR260 status-LED ownership.

## Hardware and software contract

- The default profile is high-resolution, Sinc5, 32 kSPS, four DOUT lanes,
  eight channels, and 256 frames per DMA packet.
- Current PL addresses are AXI Quad SPI `0xB0010000`, capture registers
  `0xB0020000`, and AXI DMA `0xB0030000`. When the XSA changes, use its
  implemented address map and update the driver/docs intentionally.
- Keep DMA buffers at least 64-byte aligned. The current simple S2MM path uses
  two buffers and must re-arm the next transfer before processing the completed
  packet.
- Keep high-rate work bounded. Do not perform per-frame logging or unbounded
  blocking RPMsg sends in the ADC task; the control endpoint and heartbeat must
  remain responsive.
- Linux and the RPU share a physical UART. Leave `RSPMSG_DEBUG` disabled and do
  not add routine or per-packet UART output. Prefer RPMsg health/status queries.

## Cross-repository ABI

- `common/include/rpu_control_protocol.h` defines the RPU side of the wire ABI;
  its APU counterpart is `MSAP1_APU/include/msap1/rpu_control_protocol.h`.
- Keep numeric values and packed layouts compatible and update both repositories
  together. Preserve compatibility or increment the protocol version with
  explicit handling on both peers.
- Keep the OpenAMP platform glue as C. Do not force the C compiler to use C++
  semantics; the application may link with the C++ linker.

## Build and verification

Use AMD Vitis 2025.2 from the repository root:

```sh
vitis -s scripts/build_r5_apps.py -- r5c0
vitis -s scripts/build_r5_apps.py -- r5c1
vitis -s scripts/build_r5_apps.py -- all
```

When the PL XSA or platform domains change, recreate the platform first:

```sh
mkdir -p /tmp/xilinx-vitis-data
export XILINX_VITIS_DATA_DIR=/tmp/xilinx-vitis-data
vitis -s scripts/create_platform_from_xsa.py -- --force
```

- Do not hand-edit generated `platform/`, BSP, export, or workspace metadata.
- After ADC/RPMsg changes, build R5c0 and execute the target procedure in the
  APU repository. Confirm SPI health, increasing DMA counters, zero overflow,
  a responsive control endpoint, and a continuing heartbeat.

## Maintaining this file

- Update this `AGENTS.md` in the same change when durable ownership, ABI,
  hardware addresses, build, or verification conventions change.
- Keep temporary bring-up observations in `docs/` or the cross-repository test
  procedure, not in this file.
