# ADC Health-Poll Hardening (SPI False-Alarm Suppression)

Status: planned, not implemented. Priority: low (nuisance-level today), but
should land before any alerting/fleet-monitoring consumes
`rpu_health_degraded` events.

## Context — why this plan exists

The AD7771's SPI **control/status** interface on the current hardware is
marginal, while its **measurement data** interface is demonstrably clean
(frames continuous at the configured DRDY rate, zero header errors, zero
FIFO overflows across days of soak). Observed manifestations, 2026-08-13
through 2026-08-15 on device 172.30.19.99:

1. Chronic `ADC SPI register read recovered after retry` events, roughly
   every 5–15 minutes, continuously, across multiple boots and two images.
2. `RPU ADC health became degraded: ADC register readback does not match
   the active configuration` (2026-08-15 01:40:37), recovered at the next
   poll 30 s later. Capture and configuration were provably intact.
3. `RPU ADC health became degraded: ADC INIT_COMPLETE is not asserted`
   (2026-08-15 02:35:10). The journal shows an SPI retry-recovery event
   **45 µs before** this degradation — the poll trusted whatever the
   retried transaction returned. If INIT_COMPLETE had genuinely dropped,
   the ADC would have lost its register file and stopped clocking data;
   instead DRDY continued uninterrupted and the next poll read healthy.

Diagnosis: the single-retry read policy converts a marginal SPI bus into
false health alarms. A retry that "succeeds" can still return corrupted
data, and the poller currently believes catastrophic claims from a single
read. These false degradations pollute the journal, will eventually cry
wolf during a real event, and — as happened during the 2026-08-14 record
investigation — cost engineer time ruling them in or out of unrelated
incidents.

## Code anchors

- `common/src/ad7771.cpp` — `read_adc_register()` retry logic
  (`spi_register_retry_delay_us = 10`, `retry_recovery_count`,
  `protocol_error_count`, `last_failed_register`/`last_received_header`
  in `spi_health_diagnostics_`); `wait_for_initialization()`
  (`status_init_complete`, STATUS_3); register-health snapshot
  (`read_register_health`, ~line 609).
- `R5c0/MainApp/handlers/adc/adc_health.cpp` — assembles the
  `MSAP1_ADC_HEALTH_*` bitmask sent to the APU.
- `common/include/rpu_control_protocol.h` — health flag definitions
  (`MSAP1_ADC_HEALTH_INIT_COMPLETE`, `_CONFIG_MATCH`, …).
- APU side renders the degraded reasons (`health_monitor.cpp`) from the
  bitmask; the ~30 s poll cadence is APU-observed.

## Plan

### 1. Confirm-before-degrade ("believe twice for bad news")

For any poll result that would CLEAR a health bit with catastrophic
implications — `INIT_COMPLETE`, `CONFIG_MATCH`, `SPI_RESPONSIVE`,
`INITIALIZED` — do not report it from a single read sequence. Re-read the
implicated register(s) after a short settle delay (≥ 1 ms, well beyond the
10 µs retry spacing so a bus disturbance has passed) and require **two
consecutive consistent bad reads** before the bit is cleared in the
reported bitmask. Transitions to healthy continue to apply immediately.
Detection latency for a genuine fault grows by one confirmation read
(microseconds) — not by a poll period — so the trade-off is negligible.

### 2. Status-read integrity at the source

`read_adc_register()` currently accepts the first reply whose protocol
header validates. For *status/health* reads specifically (not the
configuration write-verify path, which has its own read-back), adopt
read-twice-compare: two consecutive reads must agree, otherwise count a
`status_read_mismatch` and retry the pair. This catches the failure mode
observed on 2026-08-15: a corrupted reply with a plausible header.

### 3. Make the marginality visible instead of log-only

`spi_health_diagnostics_` (retry recoveries, protocol errors, last failed
register/header, plus the new mismatch counter from step 2) should travel
in the health payload to the APU and surface in `mnc meter health` /
`GET /api/v1/meter/health`. Today the only trace is journal lines; a
counter trend is what lets the fleet distinguish "chronic but stable
marginality" from "degrading toward failure", and it gives the
signal-integrity investigation (below) a quantitative baseline.

### 4. Flap accounting

Count degraded→healthy→degraded transitions per boot on the RPU and
include the count in the health payload. A rising flap count with clean
data-path counters is the signature of this SPI issue; the APU can then
rate-limit its warning logs on that basis.

## Companion hardware ticket (separate, not this plan)

The underlying SPI signal integrity deserves its own investigation: scope
the SCLK/SDO lines at the ADC, review the SPI clock rate and any series
termination/pull configuration, and check for correlation between retry
events and board activity (the retries have shown no correlation with
temperature, grid conditions, or system load in the 08-13..08-15 data).
The firmware hardening above is worth doing regardless of the hardware
outcome — a health reporter should not propagate single-read claims over
a link with a known error rate.

## Verification

1. Unit-level: exercise the confirm-before-degrade state machine with an
   injected flaky-read fake (bad-then-good, bad-then-bad, alternating) and
   assert: single corrupt reads never clear a bit; two consistent bad
   reads always do; recovery is immediate.
2. On-target soak (≥ 24 h): with the chronic retry rate unchanged
   (~5–15 min cadence), expect **zero** `rpu_health_degraded` events from
   SPI corruption while `retry_recovery_count` and the new mismatch
   counter keep climbing; verify via `journalctl -u
   msap1-fpga-acquisition | grep 'RPU ADC health'`.
3. Fault injection: hold the ADC in reset (or pull RESET) and confirm a
   genuine INIT_COMPLETE loss is still reported within one poll period
   plus one confirmation read.

## Non-goals

- No change to the measurement data path, capture control, or the
  configuration write-verify sequence.
- No masking of *persistent* mismatches: anything consistent across two
  reads is reported exactly as today.
- The PL record-emission fault (see
  `MSAP1_DOC/raw_doc/incident_logs/2026-08-14/ANALYSIS.md`) is unrelated
  to this plan; correlation was tested and excluded (health flaps occur
  with a fully healthy meter stream, and episodes run without flaps at
  their boundaries).
