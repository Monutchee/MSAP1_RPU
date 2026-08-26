#ifndef AGGREGATION_ENGINE_HPP
#define AGGREGATION_ENGINE_HPP

#include <hls_stream.h>

#include "agg_block_result.hpp"
#include "measurement_record.hpp"
#include "metering_types.hpp"
#include "single_cycle_packet.hpp"

// Compile-time switch for the non-normative live 10-minute and 2-hour
// previews. Completed records are unaffected; production keeps previews
// enabled.
#ifndef MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
#define MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS 1
#endif

#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS != 0 && \
    MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS != 1
#error "MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS must be 0 or 1"
#endif

// The cycle-block aggregation engine (normative source) — ONE engine
// owning both finalized tiers of the metrology chain (roadmap A1). It
// replaces Agg10_12CycleEngine and Agg150_180CycleEngine, which were
// near-identical: each carried a PRIVATE copy of the whole
// metrology_finalize.hpp arithmetic, together 77 kLUT and 406 DSP at a
// combined duty cycle of ~0.1%, and a fourth tier (M13) built the same
// way would have pushed the design past the K26's own LUT limit.
//
// One engine owns all tiers so they share the same finalize arithmetic and
// state. Two consequences follow:
//   - the 7072-bit agg_block_result beat that used to travel between the
//     tiers becomes an INTERNAL VARIABLE. Its AXIS master/slave pair, its
//     FIFO, and meter_agg150_180_hls_shim.vhd all disappear.
//   - POWER/PHASOR/UNBAL and their AGG siblings have IDENTICAL payload
//     maps (M11), so one emitter each serves both tiers, parameterized by
//     format word and period.
//
// AGGREGATION TOPOLOGY — a TREE, not a chain (IEC 61000-4-30 §4.4):
//
//     single-cycle --> 10/12 --> 150/180 (3 s)
//                            \-> 10 min --> 2 h
//
// The 3 s AND the 10 min tiers both aggregate the 10/12-CYCLE blocks,
// independently and in parallel. Only the 2 h tier aggregates another
// aggregate (the 10 min one). Chaining 10 min off the 3 s tier would be
// the natural-looking mistake and it does NOT conform: the standard
// aggregates the 10 min interval from the basic 10/12-cycle measurements
// directly, so a block feeds two accumulator sets, never one.
//
// Consequences for this engine:
//   - four accumulator sets, not a pipeline of one: blk (10/12), a3s
//     (150/180), a10m, a2h. A closing block adds itself into BOTH a3s and
//     a10m; a closing 10 min interval adds itself into a2h.
//   - the finalize loop runs up to FOUR passes on one input packet, since a
//     single block can close 10/12 and 3 s and 10 min and 2 h together.
//   - the 10 min boundary is NOT a block count. It is clock-aligned to
//     absolute time (the Class-A requirement), so that tier closes on an
//     external mark, not on "3000 blocks". M13 owns that; the pass loop
//     here must not assume a fixed divisor.
//
// Width notes for the longer tiers (M13/M14 must re-check, and A3 must
// not tighten past them): the 3 s tier sums 15 blocks, but 10 min at
// 60 Hz sums ~3000 blocks (~76.8 M samples at 128 kSPS) and 2 h sums 12
// of those (~921.6 M samples). The u128 square accumulator still holds a
// realistic signal there (~2^80), but the SATURATING add and its sticky
// flag are what bound the pathological full-range case, and they trip
// ~200x sooner at 10 min than at 3 s. The 32-bit sample count is the
// harder limit: 921.6 M fits, but it overflows at ~33.5 M samples/s x
// 268 s... i.e. ~9.3 hours at 128 kSPS, so any tier longer than 2 h
// needs a 64-bit count.
//
// The finalize has exactly ONE call site — the four-pass loop in the
// engine body, pass 0 closing a block, pass 1 closing the 150/180-cycle
// interval, pass 2 closing the clock-aligned 10-minute interval, and pass 3
// closing twelve complete 10-minute intervals into the 2-hour result —
// so it is one hardware instance whatever the inliner decides. The
// accumulator set for the pass is selected by a ROLLED copy loop rather
// than a parallel mux: ~1.3 kLUT of selection instead of ~9 k.
//
//   s_result : one fixed packet per whole grid cycle. The first 221
//              32-bit words carry single_cycle_result_t; the PL exporter
//              appends 13 context words. This replaces the former 7,488-bit
//              beat without changing measurement content.
//   m_basic  : the 10/12-cycle record quad, back to back, each MREC_WORDS
//              x 32 beats with TLAST on the last and shared correlation
//              fields (sequence, generation, anchors, block status):
//                BASIC-v4  (0x00010004), POWER-v1  (0x00070001),
//                PHASOR-v2 (0x00080002), UNBAL-v2  (0x00090002).
//   m_agg    : the aggregate record quads on the same rules. The 150/180
//              tier's quad today; the 10 min and 2 h quads join it on this
//              same master (word 1 disambiguates, the
//              house rule for multiple formats on one stream):
//                AGG-v3        (0x00020003), AGG-POWER-v1  (0x00100001),
//                AGG-PHASOR-v2 (0x00110002), AGG-UNBAL-v2  (0x00120002).
//                TENMIN-v1     (0x000C0001), TENMIN-POWER  (0x00130001),
//                TENMIN-PHASOR (0x00140002), TENMIN-UNBAL  (0x00150002).
//                TWOHOUR-v1    (0x000D0001), TWOHOUR-POWER (0x00160001),
//                TWOHOUR-PHASOR(0x00170002), TWOHOUR-UNBAL (0x00180002).
//              Emitted on every 15th eligible block, so ~1 quad per 3 s
//              against the basic tier's ~5 quads per second.
//
// Tier rules are unchanged from the engines this replaces and stay
// normative there in spirit: the block contract (N consecutive whole
// cycles under one generation and nominal, APPLY latching, lock/fallback
// reduction, sticky arithmetic flag) and the interval contract (15
// consecutive eligible blocks, sequence and sample continuity, only
// complete intervals emitted, the AGG_* diagnostic counters).

// ---------------------------------------------------------------------------
// Input packet: SCYC_PACKET_WORDS SingleCycleResult words followed by the
// PL-exported context below. The word positions are a private lock-step
// contract shared with the PL exporter and the R5C1 frame decoder.
// ---------------------------------------------------------------------------
static const int AGG_CONTEXT_CFG_GEN_WORD = 0;
static const int AGG_CONTEXT_CFG_RATE_WORD = 1;
static const int AGG_CONTEXT_CONTROL_WORD = 2;
static const int AGG_CONTEXT_FREQ_STATUS_WORD = 3;
static const int AGG_CONTEXT_FREQ_PERIOD_WORD = 4;
static const int AGG_CONTEXT_FREQ_SEQ_WORD = 5;
static const int AGG_CONTEXT_CAP_FRAMES_WORD = 6;
static const int AGG_CONTEXT_CAP_HDRERR_WORD = 7;
static const int AGG_CONTEXT_CAP_OVERFLOW_WORD = 8;
static const int AGG_CONTEXT_CAP_ALERTS_WORD = 9;
static const int AGG_CONTEXT_TEN_MIN_TARGET_LOW_WORD = 10;
static const int AGG_CONTEXT_TEN_MIN_TARGET_HIGH_WORD = 11;
static const int AGG_CONTEXT_TARGET_CONTROL_WORD = 12;
static const int AGG_CONTEXT_WORDS = 13;
static const int AGG_INPUT_PACKET_WORDS = SCYC_PACKET_WORDS + AGG_CONTEXT_WORDS;

static const int AGG_CONTEXT_MASK_LSB = 0;
static const int AGG_CONTEXT_ENABLE_BIT = 8;
static const int AGG_CONTEXT_DC_REMOVE_BIT = 9;
static const int AGG_CONTEXT_APPLY_BIT = 10;
static const int AGG_CONTEXT_LOCKED_BIT = 11;
static const int AGG_CONTEXT_FALLBACK_BIT = 12;
static const int AGG_CONTEXT_TARGET_VALID_BIT = 0;
static const int AGG_CONTEXT_TARGET_UPDATE_BIT = 1;

static_assert(AGG_INPUT_PACKET_WORDS == 234,
              "aggregation packet layout changed; update the PL/R5 contract");

void hls_aggregation_engine(hls::stream<single_cycle_word_t> &s_result,
                            hls::stream<record_axis_t> &m_basic,
                            hls::stream<record_axis_t> &m_agg);

// R5C1 uses this scheduling hook to drain only real deferred interval work
// instead of repeatedly invoking the arbitrary-precision datapath.
bool hls_aggregation_engine_has_pending_work();

#endif  // AGGREGATION_ENGINE_HPP
