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
  1 has no ADC or KR260 status-LED ownership. R5 core 1 is the production
  interval-aggregation authority: it receives the PL SingleCycle packet,
  finalizes all interval tiers using the RPU-owned implementation under
  `R5c1/src/MainApp/aggregation/`, and returns complete meter records to PL.

## Hardware and software contract

- The default profile is high-resolution, Sinc5, 128 kSPS, four DOUT lanes,
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
- PL aggregate health registers `0x78`-`0x8C` remain read-only status inputs.
  One AXI FIFO MM-S carries four exact PL-to-R5C1 packet contracts: 239-word
  AGG1, 69-word PQE1, 1,043-word VSB1 revision 1, and 2,693-word HRM1. VSB1
  contains ten metadata words, 256 consecutive converted VA/VB/VC samples as
  signed 32-bit microvolts plus one frame-status word, and four trailer words.
  Flicker and mains signalling share this one voltage batch; do not duplicate
  the raw stream or restore a PL-side mains observation packet. Arbitration
  occurs only at whole-packet boundaries. The
  dedicated validator dispatches by magic and must reject a mixed-image
  contract, malformed family geometry, inconsistent provenance, or CRC error.
  These are exact co-release interfaces, not negotiated protocol versions;
  no legacy decoder is maintained. R5C1 owns Basic, 150/180-cycle,
  10-minute, and 2-hour aggregation for ordinary measurements, plus
  RMS-magnitude plus magnitude-weighted circular-phase 150/180-cycle, UTC
  10-minute, and 2-hour harmonic aggregation. R5C1 also owns Flicker reference
  normalization, 2 kHz decimation, IEC 61000-4-15 filtering and
  classification, Pst, Plt, and the unchanged `0x000E0001` public record. It
  also owns the seven-probe 200 ms mains-signalling estimator and unchanged
  `0x000F0001` public record; do not restore either retired PL HLS engine.
  It writes complete 256-byte records into the same FIFO's TX side. Aggregate
  measurement data never travels over RPMsg; the FIFO return stream joins the
  existing meter AXIS switch and Linux DMA path. The PL direct harmonic output
  remains only as a temporary target-proof fallback and is not another
  aggregation authority.
- At each valid UTC ten-minute sample target, R5C1 starts one transient Basic
  shadow on the first whole cycle at or after the target while the old Basic
  completes. The first synchronized Basic carries timing-word bit 19. That
  Basic similarly seeds one transient 150/180-cycle shadow while the old
  15-Basic interval completes; aggregate status bit 3 marks the continuing
  overlap and bit 4 marks the synchronized interval. Keep exactly two slots
  per affected tier, promote by selector, and continue using the one shared
  finalizer. AGG-v3 words 36/37 are the actual last contributing sample, so
  only a bit-3 overlap record may have a physical range shorter than its
  summed sample count.
- R5C1 owns volatile M17 energy and demand session arithmetic. ENERGY-v1
  (`0x00030001`) is an atomic two-record family: summary then quadrants, with
  matching sequence, generation, session, anchors, and family count. Reactive
  energy uses the shared `EnergyQuadrant` classifier and fundamental Q1: I is
  `P>=0,Q1>0`, II is `P<0,Q1>0`, III is `P<0,Q1<0`, and IV is
  `P>=0,Q1<0`; `Q1==0` contributes to no quadrant. DEMAND-v1
  (`0x00040001`) is active-only and carries its method, window, update cadence,
  and profile generation. The default is a 60-second sliding average emitted
  on each completed 150/180-cycle aggregate (nominally every 3 seconds);
  supported sliding windows are 60, 300, 600, 900, and 1800 seconds. Fixed
  mode remains one aligned UTC 600-second block emitted at its boundary. These
  profiles are M17 product policy, not an IEC 61000-4-30 active-demand
  requirement. A rejected sliding bucket clears the window; its incomplete
  quality clears after one full clean refill rather than latching for the
  session. Apply profile changes only at aggregation record boundaries and
  reset demand-window/session-peak state without touching energy state.
  Linux owns lifetime totals and reset epochs; APPLY and invalid blocks never
  reset the R5C1 session. Generate the nonzero boot session ID
  only after BSP/runtime constructors complete, mixing the SoC-wide system
  counter with the R5-local cycle counter; never derive it from deterministic
  static-constructor timing or addresses.
- Keep the M17 engine and all future large accumulator state in static
  `.bss`/`.data`, clear it in place, and keep it non-copyable. Processing paths
  must not allocate dynamically or place whole engines/large arrays on a task
  stack. Retain fixed-point division remainders and saturate public counters
  instead of wrapping. Before the first fully valid coherent Basic family,
  treat input as startup priming: do not integrate it, emit ENERGY, or count it
  as skipped session time. After that baseline is established, preserve sticky
  saturation, discontinuity, and incomplete-input provenance for every real
  runtime rejection.
- Physical and simulated ADC sources share the raw PL stream boundary. Stop
  capture before switching sources and commit a source change only after
  target-device configuration and PL readback succeed. While simulation is
  active, do not access physical ADC SPI and report its diagnostics as not
  applicable rather than unhealthy.
- ADC samples and meter results never travel over RPMsg. RPMsg is limited to
  START, STOP, runtime RMS/frequency configuration, and health/control traffic so the
  endpoint and heartbeat stay responsive.
- The aggregation FIFO ISR only acknowledges the device and notifies the
  high-priority input task. FIFO ownership, complete-frame buffering, packet
  validation, aggregation state, record formatting, output-ring ownership,
  and health reporting remain separate classes under
  `R5c1/src/MainApp/aggregation/`. Do not place parsing or arithmetic in the
  ISR or collapse the offload into `main.cpp`. The RX worker must retain its
  bounded four-packet drain and one-tick validator handoff; an unbounded drain
  starves the lower-priority validator and eventually fills both hardware and
  software input storage. Production firmware must require the FIFO interrupt
  rather than silently falling back to polling. The static transport frame is
  sized to the 2,693-word HRM1 maximum, while the AXI FIFO must hold at least
  one complete HRM1 packet. VSB1 arrives at 500 packets/s for the default
  128-kSPS profile, so retain the bounded four-packet drain and one-tick
  validator handoff and configure R5C1 for a 1 kHz FreeRTOS tick. The scheduler
  rate-budget guard must cover all 685 packets/s of coincident private traffic
  with at least ten percent margin; the default 100 Hz tick is insufficient.
  Keep large frames, Flicker classifier state, and mains
  correlation banks out of worker-task stacks; all power-quality state is
  static `.bss`/`.data` and must not allocate dynamically. Mains signalling
  validates the full raw stream but bounds its seven-probe correlation at
  32 kSPS, which preserves the characterized sub-12.5-kHz analogue band and
  must remain fast enough to run concurrently with Flicker at 128 kSPS.
- ADC health reports both the measured DCLK rate and the physical
  `ADC_DRDY_N` falling-edge rate. Keep these fields coordinated with the APU
  wire-ABI copy when extending capture diagnostics. Health is not valid until
  measured DRDY matches the configured sample rate within tolerance.
- Runtime sample-rate changes arrive in `METER_CONFIG_SET` while capture is
  stopped. Apply ADC PGA/SRC and PL window configuration as one coordinated
  operating-point transaction; the packaged boot default is 128 kSPS.
  The RPMsg-v10 meter payload is 352 bytes and includes simulator-v1.5 AM and
  carrier controls; the separate fixed M18 policy payload is 316 bytes. Both
  remain below the 384-byte control-frame bound and share one coordinated,
  nonzero configuration generation. `nominal_frequency_hz` remains 50 or 60
  only and drives the grid-timing registers above; keep the wire header
  byte-identical with the APU copy when it changes.
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
- The canonical contract reserves 8 MiB per R5 firmware image. R5C0 is
  `0x3ED80000..0x3F57FFFF` and R5C1 is
  `0x3F688000..0x3FE87FFF`; each `psu_r5_ddr_0_memory_0` linker region must
  match its contract start and size exactly. Move vrings and the 1 MiB RPMsg
  buffer only through the canonical contract generator, never by hand-editing
  a generated consumer.

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
- Keep `-fstack-usage` enabled for R5C1. Every guarded M17/M18 aggregation
  frame must remain below 1 KiB, the post-link gate must leave at least 1 MiB
  in each firmware reservation, and named accumulator objects must resolve to
  `.bss` or `.data`. Target acceptance additionally requires at least 2 KiB
  measured task-stack high-water headroom.
- After ADC/RPMsg changes, build R5c0 and execute the target procedure in the
  APU repository. Confirm SPI health, meter DMA progress, matching
  configuration generations, zero overflow, a responsive control endpoint,
  and a continuing heartbeat.
- Run `bash R5c1/tests/run_aggregation_shadow_tests.sh` after changing the
  private aggregation packet, decoder, ring, CRC32C, continuity, or health
  logic, including HRM1 or harmonic interval state. Its accelerated harmonic
  case must retain one contaminated startup 10-minute family, twelve clean
  10-minute families, and their one clean 2-hour result. A refreshed XSA is
  required before validating the XLlFifo hardware path. The historical
  test-script name is retained, but production firmware emits complete
  records. A stale XSA may keep RPMsg alive for diagnostics, but ordinary
  aggregation must report unhealthy and has no PL fallback.
- Run `bash R5c1/tests/run_aggregation_engine_reference_tests.sh` after
  changing aggregation arithmetic, interval state, record serialization, or
  live previews. It runs the recovered exact-golden engine suite with previews
  both enabled and disabled, then byte-compares every completed record. Set
  Basic and 150/180-cycle UTC-overlap checks at 50/60 Hz are permanent parts
  of this suite. Set `MNC_REQUIRE_M15_INVALIDATION_MATRIX=1` to exercise accelerated
  ten-minute contamination and clean-recovery behavior across discontinuities.

## Maintaining this file

- Update this `AGENTS.md` in the same change when durable ownership, ABI,
  hardware addresses, build, or verification conventions change.
- Keep temporary bring-up observations in `docs/` or the cross-repository test
  procedure, not in this file.
