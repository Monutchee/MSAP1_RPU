# MSAP1 RPU repository guidance

## Purpose and routing

- This repository contains the FreeRTOS applications for ZynqMP R5 cores 0
  and 1 plus their shared OpenAMP and board-control code.
- Read `docs/AD7771.md` before changing the ADC path. `AdcDevice` is the common
  lifecycle interface, `Ad7771` owns the physical device, `AdcSimulator` owns
  the PL generator, and `AdcController` performs transactional source changes.
  Application code must not duplicate device-specific SPI or AXI accesses.
- R5 core 0 owns AD7771 SPI configuration, reset/synchronization, PL capture
  control/status, the status LED heartbeat, and the APU control endpoint.
  Linux owns AXI DMA, descriptors, interrupts, and DDR sample buffers. R5 core
  1 has no ADC or KR260 status-LED ownership.

## Hardware and software contract

- The default profile is high-resolution, Sinc5, 32 kSPS, four DOUT lanes,
  eight channels, and 256 frames per DMA packet.
- Current PL addresses are AXI Quad SPI `0xB0010000`, capture registers
  `0xB0020000`, Linux-owned AXI DMA `0xB0030000`, ADC conversion registers
  `0xB0040000`, meter-processing registers `0xB0050000`, and raw-simulator
  registers `0xB0080000`. RPU code must not touch the DMA registers or
  meter-record DDR buffers.
- Meter-processing grid-timing registers: `GRID_SHADOW_CONFIG` at `0x6C`
  ([7:0] cycles per block, [15:8] nominal Hz, [16] cycle-timing enable),
  `GRID_ACTIVE_CONFIG` readback at `0x70`, and read-only `GRID_STATUS` at
  `0x74` ([0] reference locked, [15:8] cycles elapsed). The RPU derives the
  cycle count from the wire nominal frequency (50 Hz -> 10, 60 Hz -> 12; the
  count is not on the wire), always enables cycle timing, commits via the
  existing apply toggle, and verifies `GRID_ACTIVE_CONFIG` readback. The RMS
  window register `0x18` remains the PL's free-run fallback window.
- Physical and simulated ADC sources share the raw PL stream boundary. Stop
  capture before switching sources and commit a source change only after
  target-device configuration and PL readback succeed. While simulation is
  active, do not access physical ADC SPI and report its diagnostics as not
  applicable rather than unhealthy.
- ADC samples and meter results never travel over RPMsg. RPMsg is limited to
  START, STOP, runtime RMS/frequency configuration, and health/control traffic so the
  endpoint and heartbeat stay responsive.
- ADC health reports both the measured DCLK rate and the physical
  `ADC_DRDY_N` falling-edge rate. Keep these fields coordinated with the APU
  wire-ABI copy when extending capture diagnostics. Health is not valid until
  measured DRDY matches the configured sample rate within tolerance.
- Runtime sample-rate changes arrive in `METER_CONFIG_SET` while capture is
  stopped. Apply ADC PGA/SRC and PL window configuration as one coordinated
  operating-point transaction; the packaged boot default remains 32 kSPS.
  The 176-byte version-2 config payload ends with `nominal_frequency_hz`
  (50 or 60 only), which drives the grid-timing registers above; keep the
  wire header byte-identical with the APU copy when it changes.
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
- Build OpenAMP policy from the manifest-owned `openamp-contract.json`.
  `openamp_contract.h` supplies only shared-memory and mailbox policy;
  the XSA-generated `xparameters.h` remains authoritative for peripheral
  addresses and interrupt assignments. Do not reintroduce a machine-conf or
  BSP-generated-header dependency into the RPU build.

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
  APU repository. Confirm SPI health, meter DMA progress, matching
  configuration generations, zero overflow, a responsive control endpoint,
  and a continuing heartbeat.

## Maintaining this file

- Update this `AGENTS.md` in the same change when durable ownership, ABI,
  hardware addresses, build, or verification conventions change.
- Keep temporary bring-up observations in `docs/` or the cross-repository test
  procedure, not in this file.
