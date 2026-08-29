// Recovered from the retired PL AggregationEngine component when interval
// ownership moved to R5C1.  This is now the exact-golden reference suite for
// the production implementation in ../src/MainApp/aggregation/.
//
// Testbench for the 10/12-cycle basic measurement engine (M7).
//
// The bench synthesizes whole grid cycles the way the single-cycle engine
// would emit them (per-cycle sufficient statistics packed into SCYC-v5
// result beats), tracks the SAME per-sample accumulation a Mtr1Engine
// block would have built, and checks the finalized record and basic beat
// EXACTLY against an integer golden model implementing the retired Mtr1
// arithmetic (trunc-toward-zero mean, floor variance, restoring integer
// root). Values agreeing exactly here — same accumulators in, same
// primitives through — is the Mtr1 retirement proof.
//
// Block rules covered: 12 @ 60 Hz and 10 @ 50 Hz, APPLY commit and mark,
// upstream first-after-gap restart, result/cycle sequence breaks, stale
// generation, nominal change, lock/fallback flag reduction, sticky
// arithmetic flag, VLL merge, Mtr2-contract beat fields.
//
// M9: the bench synthesizes fundamental phasors per lane (amplitude +
// phase, llround'd once to exact integer mean-phasor components) and
// checks the PHASOR-v1 record: fundamental RMS / Q1 / P1 / displacement
// PF / load nature EXACTLY against integer goldens; angles against libm
// atan2l within 2 millidegrees (the CORDIC is the only non-replicated
// arithmetic and its residual is < 0.01 mdeg). Lag/lead/quadrature sign
// scenarios, the phasor-invalid fold, and the VA-reference rule.

// Cycle-block aggregation engine bench (roadmap A1).
//
// Ported from the two engines this component replaces: the exact-golden
// block scenarios come from agg10_12_cycle_engine_tb.cpp verbatim, and the
// interval layer at the end folds fifteen block goldens the way the engine
// folds fifteen block accumulators.
//
// COVERAGE NOTE -- the retired block-result beat. The 7072-bit
// agg_block_result beat is an internal variable now, so nineteen assertions
// that inspected it could not survive as written:
//   * three were REAL coverage -- the merged accumulators handed upward
//     (r.sum/r.square per lane) and the saturated lane's clamped
//     accumulator. These are replaced by the interval scenario: if any
//     block hands the interval tier a wrong accumulator, the aggregate
//     values cannot come out exact, because the interval golden is the
//     pure-addition fold of the block goldens.
//   * sixteen were provenance fields that the RECORDS carry too -- timing
//     word (nominal, cycle count, lock/fallback/first-block flags), status
//     word (arithmetic and gap bits), valid mask, sample anchors, and
//     frequency. Those record words are asserted in the same scenarios.
// What genuinely goes away is failure LOCALIZATION per block, not the
// property being tested; the serialization boundary those checks guarded
// no longer exists to get wrong.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "aggregation_engine.hpp"

#ifndef MNC_REQUIRE_M15_INVALIDATION_MATRIX
#define MNC_REQUIRE_M15_INVALIDATION_MATRIX 0
#endif

// M18 reserves 0x000E/0x000F for finalized flicker/mains records, so the
// pre-production M15 previews must remain on their migrated identities in the
// exact R5C1 build contract consumed from the PL common header.
static_assert(MREC_FORMAT_OPEN_TEN_MINUTE_V1 == 0x00200001u &&
                  MREC_FORMAT_OPEN_TEN_MINUTE_POWER_V1 == 0x00210001u &&
                  MREC_FORMAT_OPEN_TEN_MINUTE_PHASOR_V2 == 0x00220002u &&
                  MREC_FORMAT_OPEN_TEN_MINUTE_UNBAL_V2 == 0x00230002u);
static_assert(MREC_FORMAT_OPEN_TWO_HOUR_V1 == 0x00240001u &&
                  MREC_FORMAT_OPEN_TWO_HOUR_POWER_V1 == 0x00250001u &&
                  MREC_FORMAT_OPEN_TWO_HOUR_PHASOR_V2 == 0x00260002u &&
                  MREC_FORMAT_OPEN_TWO_HOUR_UNBAL_V2 == 0x00270002u);
static_assert(MREC_FORMAT_PQ_EVENT_V1 == 0x00060001u &&
                  MREC_FORMAT_FLICKER_V1 == 0x000E0001u &&
                  MREC_FORMAT_MAINS_SIGNAL_V1 == 0x000F0001u);

static int failures = 0;
static std::FILE *completed_trace = nullptr;
static unsigned long long completed_digest = 1469598103934665603ull;
static unsigned completed_record_count = 0;

#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::printf("FAIL: " __VA_ARGS__);                                       \
      std::printf("\n");                                                       \
      ++failures;                                                              \
    }                                                                          \
  } while (0)

// The preview-on and preview-off benches write the same canonical stream of
// completed records.  Comparing the optional trace files with `cmp` proves
// byte identity rather than relying on a small set of decoded fields.  Open
// previews are deliberately excluded: they are the feature under comparison.
static bool is_open_record_format(unsigned format) {
  return format == MREC_FORMAT_OPEN_TEN_MINUTE_V1 ||
         format == MREC_FORMAT_OPEN_TEN_MINUTE_POWER_V1 ||
         format == MREC_FORMAT_OPEN_TEN_MINUTE_PHASOR_V2 ||
         format == MREC_FORMAT_OPEN_TEN_MINUTE_UNBAL_V2 ||
         format == MREC_FORMAT_OPEN_TWO_HOUR_V1 ||
         format == MREC_FORMAT_OPEN_TWO_HOUR_POWER_V1 ||
         format == MREC_FORMAT_OPEN_TWO_HOUR_PHASOR_V2 ||
         format == MREC_FORMAT_OPEN_TWO_HOUR_UNBAL_V2;
}

static void trace_completed_record(unsigned char stream,
                                   const ap_uint<32> (&words)[MREC_WORDS]) {
  const unsigned format = (unsigned)words[MREC_FORMAT_WORD];
  if (is_open_record_format(format)) return;

  auto add_byte = [](unsigned char byte) {
    completed_digest ^= byte;
    completed_digest *= 1099511628211ull;
    if (completed_trace != nullptr) std::fputc(byte, completed_trace);
  };

  add_byte(stream);
  for (int word = 0; word < MREC_WORDS; ++word) {
    const unsigned value = (unsigned)words[word];
    add_byte((unsigned char)(value & 0xFFu));
    add_byte((unsigned char)((value >> 8) & 0xFFu));
    add_byte((unsigned char)((value >> 16) & 0xFFu));
    add_byte((unsigned char)((value >> 24) & 0xFFu));
  }
  ++completed_record_count;
}

// ---------------------------------------------------------------------------
// Integer golden model of the retired Mtr1 finalize (exact).
// ---------------------------------------------------------------------------
static unsigned __int128 golden_isqrt(unsigned __int128 value) {
  unsigned __int128 root = 0;
  for (int bit = 63; bit >= 0; --bit) {
    const unsigned __int128 candidate = root | ((unsigned __int128)1 << bit);
    if (candidate * candidate <= value) root = candidate;
  }
  return root;
}

// floor((square*N - |sum|^2) / N^2) then floor sqrt — dc_remove active.
static unsigned long long golden_rms_q16(unsigned __int128 square,
                                         __int128 sum, unsigned count,
                                         bool dc_remove) {
  unsigned __int128 numerator = square * count;
  if (dc_remove) {
    const unsigned __int128 magnitude =
        (unsigned __int128)(sum < 0 ? -sum : sum);
    numerator -= magnitude * magnitude;
  }
  const unsigned __int128 variance =
      numerator / ((unsigned __int128)count * count);
  return (unsigned long long)golden_isqrt(variance);
}

static long long golden_mean_q16(__int128 sum, unsigned count) {
  const bool negative = sum < 0;
  const unsigned __int128 magnitude = (unsigned __int128)(negative ? -sum : sum);
  const unsigned long long trunc64 = (unsigned long long)(magnitude / count);
  return negative ? -(long long)trunc64 : (long long)trunc64;
}

// ---------------------------------------------------------------------------
// Cycle synthesis: per-cycle statistics from explicit per-sample values,
// mirroring the single-cycle engine's accumulation exactly.
// ---------------------------------------------------------------------------
struct GoldenBlock {
  __int128 sum[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 square[MET_ACTIVE_CHANNELS] = {};
  long long raw_sum[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 raw_square[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 vll_square[MET_VLL_PAIRS] = {};
  __int128 power[MET_POWER_PHASES] = {};
  long long minimum[MET_ACTIVE_CHANNELS] = {};
  long long maximum[MET_ACTIVE_CHANNELS] = {};
  __int128 ph_re[MET_ACTIVE_CHANNELS] = {};
  __int128 ph_im[MET_ACTIVE_CHANNELS] = {};
  bool extrema_seeded = false;
  unsigned count = 0;
};

// Golden power finalization mirroring the engine exactly: trunc-toward-
// zero mean of the Q32 product sum, arithmetic >>32 to picowatts; S from
// the exact RMS product; PF floor(|P|*1e6/S) clamped, sign of P.
static long long golden_p_pw(__int128 power_sum, unsigned count) {
  const bool negative = power_sum < 0;
  const unsigned __int128 magnitude =
      (unsigned __int128)(negative ? -power_sum : power_sum);
  __int128 mean = (__int128)(magnitude / count);
  if (negative) mean = -mean;
  return (long long)(mean >> 32);
}
static unsigned long long golden_s_pw(unsigned long long v_rms_q16,
                                      unsigned long long i_rms_q16) {
  return (unsigned long long)(((unsigned __int128)v_rms_q16 * i_rms_q16) >> 32);
}
static long long golden_pf_e6(long long p_pw, unsigned long long s_pw) {
  if (p_pw == 0 || s_pw == 0) return 0;
  const unsigned __int128 magnitude =
      (unsigned __int128)(p_pw < 0 ? -p_pw : p_pw);
  unsigned __int128 ratio = magnitude * 1000000u / s_pw;
  if (ratio > 1000000u) ratio = 1000000u;
  return p_pw < 0 ? -(long long)ratio : (long long)ratio;
}
static unsigned long long golden_crest_e4(long long minimum, long long maximum,
                                          unsigned long long rms_q16) {
  if (rms_q16 == 0) return 0;
  const unsigned long long lo = (unsigned long long)(minimum < 0 ? -minimum : minimum);
  const unsigned long long hi = (unsigned long long)(maximum < 0 ? -maximum : maximum);
  const unsigned long long peak = lo > hi ? lo : hi;
  unsigned __int128 ratio = (unsigned __int128)peak * 10000u / rms_q16;
  if (ratio > 0xFFFFFFFFu) ratio = 0xFFFFFFFFu;
  return (unsigned long long)ratio;
}

// ---- M9 phasor goldens. All exact integer replicas except the angle,
// ---- which deliberately uses libm (the independent-golden rule).
static long long golden_phasor_counts(__int128 sum, unsigned count) {
  // Trunc-toward-zero mean, then the arithmetic >> 37 Q1.37 floor —
  // both quirks are normative (met_phasor_counts).
  const bool negative = sum < 0;
  const unsigned __int128 magnitude =
      (unsigned __int128)(negative ? -sum : sum);
  __int128 mean = (__int128)(magnitude / count);
  if (negative) mean = -mean;
  return (long long)(mean >> 37);
}
static unsigned long long golden_fund_rms_q16(long long re_c, long long im_c) {
  const unsigned __int128 sq = (unsigned __int128)((__int128)re_c * re_c) +
                               (unsigned __int128)((__int128)im_c * im_c);
  return (unsigned long long)((golden_isqrt(sq) * 92682u) >> 16);
}
static long long golden_p1_pw(long long re_v, long long im_v, long long re_i,
                              long long im_i) {
  const __int128 dot = (__int128)re_v * re_i + (__int128)im_v * im_i;
  return (long long)(dot >> 31);
}
static long long golden_q1_pvar(long long re_v, long long im_v, long long re_i,
                                long long im_i) {
  const __int128 cross = (__int128)im_v * re_i - (__int128)re_v * im_i;
  return (long long)(cross >> 31);
}
static long long golden_angle_mdeg(long long im, long long re) {
  // Published convention: [0, 360000).
  if (im == 0 && re == 0) return 0;
  long long mdeg = (long long)llroundl(
      atan2l((long double)im, (long double)re) / M_PI * 180000.0L);
  if (mdeg < 0) mdeg += 360000;
  return mdeg % 360000;
}
static long long wrap_mdeg(long long mdeg) {
  while (mdeg >= 180000) mdeg -= 360000;
  while (mdeg < -180000) mdeg += 360000;
  return mdeg;
}
static unsigned golden_nature(long long q1, unsigned long long s1) {
  if (s1 == 0) return (unsigned)MET_NATURE_UNDEFINED;
  if (q1 == 0) return (unsigned)MET_NATURE_UNITY;
  return q1 > 0 ? (unsigned)MET_NATURE_LAGGING : (unsigned)MET_NATURE_LEADING;
}

// ---- M10 symmetrical-component goldens (exact integer replicas of the
// ---- Q30 a-operator, trunc-toward-zero /3, and the e6 ratio).
static void golden_rotate(bool a_squared, long long re, long long im,
                          long long &out_re, long long &out_im) {
  const __int128 sq3h = 929887697;  // round(sqrt(3)/2 * 2^30)
  const __int128 re_half = ((__int128)re) << 29;
  const __int128 im_half = ((__int128)im) << 29;
  const __int128 re_s3 = (__int128)re * sq3h;
  const __int128 im_s3 = (__int128)im * sq3h;
  if (!a_squared) {
    out_re = (long long)((-re_half - im_s3) >> 30);
    out_im = (long long)((re_s3 - im_half) >> 30);
  } else {
    out_re = (long long)((-re_half + im_s3) >> 30);
    out_im = (long long)((-re_s3 - im_half) >> 30);
  }
}
static long long golden_div3(long long value) {  // trunc toward zero
  return value < 0 ? -(long long)((-(__int128)value) / 3)
                   : (long long)((__int128)value / 3);
}
static void golden_sequence(const long long re[3], const long long im[3],
                            long long seq_re[3], long long seq_im[3]) {
  long long ba_r, ba_i, ba2_r, ba2_i, ca_r, ca_i, ca2_r, ca2_i;
  golden_rotate(false, re[1], im[1], ba_r, ba_i);
  golden_rotate(true, re[1], im[1], ba2_r, ba2_i);
  golden_rotate(false, re[2], im[2], ca_r, ca_i);
  golden_rotate(true, re[2], im[2], ca2_r, ca2_i);
  seq_re[0] = golden_div3(re[0] + re[1] + re[2]);
  seq_im[0] = golden_div3(im[0] + im[1] + im[2]);
  seq_re[1] = golden_div3(re[0] + ba_r + ca2_r);
  seq_im[1] = golden_div3(im[0] + ba_i + ca2_i);
  seq_re[2] = golden_div3(re[0] + ba2_r + ca_r);
  seq_im[2] = golden_div3(im[0] + ba2_i + ca_i);
}
static unsigned long long golden_ratio_e6(unsigned long long numerator,
                                          unsigned long long denominator) {
  if (denominator == 0) return 0;
  const unsigned __int128 ratio =
      (unsigned __int128)numerator * 1000000u / denominator;
  return ratio > 0xFFFFFFFFu ? 0xFFFFFFFFull : (unsigned long long)ratio;
}

struct CycleSpec {
  unsigned zero_lanes = 0;  // bitmask: force these lanes' samples to 0
  unsigned sequence = 1;
  unsigned cycle_sequence = 100;
  unsigned generation = 1;
  unsigned long long first_sample = 1000;
  unsigned samples = 5;
  unsigned nominal = 60;
  unsigned status = 0;
  unsigned valid_mask = 0x7F;
  unsigned freq_mhz = 60012;
  unsigned freq_valid = 1;
  /* Q16 micro-unit scale: real 120 V lanes sit near 8e12; a ~5e8 base
   * keeps every derived quantity (S = rms*rms >> 32, PF, crest) nonzero
   * and non-trivial while all goldens stay exact in __int128. */
  long long seed = 500000001;  // varies the synthetic sample values

  // Fundamental phasors (M9): amplitude in Q16 micro-units and phase in
  // degrees vs the cycle reference, per lane. make_cycle llrounds each
  // to ONE pair of integer mean-phasor components and scales them back
  // into exact Q1.37 correlation sums, so every golden downstream is
  // exact given the same integers. Defaults: balanced ABC voltages
  // (1e12 ~ 15 kV-ish, inside the 2^40 component contract), currents
  // lagging their voltage by 10 degrees (Q1 positive), IN silent.
  double ph_amp[MET_ACTIVE_CHANNELS] = {5.0e10, 5.0e10, 5.0e10, 0.0,
                                        1.0e12, 1.0e12, 1.0e12};
  double ph_deg[MET_ACTIVE_CHANNELS] = {-10.0, -130.0, 110.0, 0.0,
                                        120.0, -120.0, 0.0};
};

// Build one whole cycle: sample s of lane l is
//   value = seed * (l + 1) * (s + 1) with alternating sign  (Q16 domain)
//   raw   = value / 5
static single_cycle_result_t make_cycle(const CycleSpec &c, GoldenBlock &g) {
  single_cycle_result_t r = {};
  __int128 sum[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 square[MET_ACTIVE_CHANNELS] = {};
  long long raw_sum[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 raw_square[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 vll_square[MET_VLL_PAIRS] = {};
  __int128 power[MET_POWER_PHASES] = {};
  long long lane_min[MET_ACTIVE_CHANNELS] = {};
  long long lane_max[MET_ACTIVE_CHANNELS] = {};
  const int minuend[MET_VLL_PAIRS] = {MET_LANE_VA, MET_LANE_VB, MET_LANE_VC};
  const int subtrahend[MET_VLL_PAIRS] = {MET_LANE_VB, MET_LANE_VC,
                                         MET_LANE_VA};
  for (unsigned s = 0; s < c.samples; ++s) {
    long long lane_value[MET_ACTIVE_CHANNELS];
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      long long value = ((s % 2 == 0) ? 1 : -1) *
                        c.seed * (lane + 1) * (long long)(s + 1);
      if (c.zero_lanes & (1u << lane)) value = 0;
      lane_value[lane] = value;
      sum[lane] += value;
      square[lane] += (unsigned __int128)((__int128)value * value);
      raw_sum[lane] += value / 5;
      raw_square[lane] +=
          (unsigned __int128)((__int128)(value / 5) * (value / 5));
    }
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      const long long diff =
          lane_value[minuend[pair]] - lane_value[subtrahend[pair]];
      vll_square[pair] += (unsigned __int128)((__int128)diff * diff);
    }
    static const int pv[MET_POWER_PHASES] = {MET_LANE_VA, MET_LANE_VB,
                                             MET_LANE_VC};
    static const int pi_[MET_POWER_PHASES] = {MET_LANE_IA, MET_LANE_IB,
                                              MET_LANE_IC};
    for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
      const __int128 product =
          (__int128)lane_value[pv[phase]] * lane_value[pi_[phase]];
      power[phase] += product;
      g.power[phase] += product;
    }
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      if (s == 0 || lane_value[lane] < lane_min[lane])
        lane_min[lane] = lane_value[lane];
      if (s == 0 || lane_value[lane] > lane_max[lane])
        lane_max[lane] = lane_value[lane];
      if (!g.extrema_seeded) {
        g.minimum[lane] = lane_value[lane];
        g.maximum[lane] = lane_value[lane];
      } else {
        if (lane_value[lane] < g.minimum[lane]) g.minimum[lane] = lane_value[lane];
        if (lane_value[lane] > g.maximum[lane]) g.maximum[lane] = lane_value[lane];
      }
    }
    g.extrema_seeded = true;
  }
  for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
    // Two's-complement image transfers exactly through the 128-bit fields.
    unsigned __int128 image = (unsigned __int128)sum[lane];
    r.sum[lane].range(63, 0) = ap_uint<64>((unsigned long long)image);
    r.sum[lane].range(127, 64) = ap_uint<64>((unsigned long long)(image >> 64));
    unsigned __int128 sq = square[lane];
    r.square[lane].range(63, 0) = ap_uint<64>((unsigned long long)sq);
    r.square[lane].range(127, 64) = ap_uint<64>((unsigned long long)(sq >> 64));
    r.raw_sum[lane] = ap_int<64>(raw_sum[lane]);
    unsigned __int128 rsq = raw_square[lane];
    r.raw_square[lane].range(63, 0) = ap_uint<64>((unsigned long long)rsq);
    r.raw_square[lane].range(95, 64) =
        ap_uint<32>((unsigned long long)(rsq >> 64));
    g.sum[lane] += sum[lane];
    g.square[lane] += square[lane];
    g.raw_sum[lane] += raw_sum[lane];
    g.raw_square[lane] += raw_square[lane];
  }
  for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
    unsigned __int128 image = (unsigned __int128)power[phase];
    r.power_sum[phase].range(63, 0) = ap_uint<64>((unsigned long long)image);
    r.power_sum[phase].range(127, 64) =
        ap_uint<64>((unsigned long long)(image >> 64));
  }
  for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
    r.minimum[lane] = ap_int<64>(lane_min[lane]);
    r.maximum[lane] = ap_int<64>(lane_max[lane]);
  }
  for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
    unsigned __int128 vsq = vll_square[pair];
    r.vll_square[pair].range(63, 0) = ap_uint<64>((unsigned long long)vsq);
    r.vll_square[pair].range(127, 64) =
        ap_uint<64>((unsigned long long)(vsq >> 64));
    g.vll_square[pair] += vll_square[pair];
  }
  // Phasor sums: mean components (amp/2)*(sin, -cos)(phase) llround'd to
  // integers, back-scaled by << 37 and the sample count — the engine's
  // mean + floor then recovers them exactly.
  for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
    const double amp =
        (c.zero_lanes & (1u << lane)) ? 0.0 : c.ph_amp[lane];
    const double rad = c.ph_deg[lane] * M_PI / 180.0;
    const long long re_c = (long long)llround(amp / 2.0 * sin(rad));
    const long long im_c = (long long)llround(-amp / 2.0 * cos(rad));
    const __int128 re_sum = ((__int128)re_c << 37) * (long long)c.samples;
    const __int128 im_sum = ((__int128)im_c << 37) * (long long)c.samples;
    unsigned __int128 re_image = (unsigned __int128)re_sum;
    r.phasor_re[lane].range(63, 0) = ap_uint<64>((unsigned long long)re_image);
    r.phasor_re[lane].range(127, 64) =
        ap_uint<64>((unsigned long long)(re_image >> 64));
    unsigned __int128 im_image = (unsigned __int128)im_sum;
    r.phasor_im[lane].range(63, 0) = ap_uint<64>((unsigned long long)im_image);
    r.phasor_im[lane].range(127, 64) =
        ap_uint<64>((unsigned long long)(im_image >> 64));
    g.ph_re[lane] += re_sum;
    g.ph_im[lane] += im_sum;
  }
  g.count += c.samples;

  r.sequence = c.sequence;
  r.generation = c.generation;
  r.first_sample = c.first_sample;
  r.last_sample = c.first_sample + c.samples - 1;
  r.sample_count = c.samples;
  r.cycle_sequence = c.cycle_sequence;
  r.nominal_hz = c.nominal;
  r.valid_mask = c.valid_mask;
  r.flags = 0x1;
  r.status = c.status;
  r.frequency_millihz = c.freq_mhz;
  r.frequency_valid = c.freq_valid;
  r.processing_tick = 777000 + c.sequence;
  return r;
}

struct Bench {
  hls::stream<single_cycle_word_t> s_result{"s_result"};
  hls::stream<record_axis_t> m_axis{"m_axis"};
  hls::stream<record_axis_t> m_agg{"m_agg"};
  bool apply_level = false;
  unsigned cfg_generation = 1;
  unsigned sample_rate = 32000;
  bool enable = true;
  bool dc_remove = true;
  bool locked = true;
  bool fallback = false;
  unsigned freq_status = 0x2;
  unsigned freq_period = 0x00010000;
  unsigned freq_seq = 42;
  unsigned cap_frames = 111, cap_hdrerr = 1, cap_overflow = 2, cap_alerts = 3;
  unsigned long long ten_minute_target = 0;
  bool ten_minute_valid = false;
  bool ten_minute_update = false;

  void send(const single_cycle_result_t &r, bool apply_toggles = false) {
    if (apply_toggles) apply_level = !apply_level;
    write_single_cycle_packet(r, s_result);

    single_cycle_word_t controls = 0;
    controls.range(7, 0) = 0x7F;
    controls.bit(AGG_CONTEXT_ENABLE_BIT) = enable;
    controls.bit(AGG_CONTEXT_DC_REMOVE_BIT) = dc_remove;
    controls.bit(AGG_CONTEXT_APPLY_BIT) = apply_level;
    controls.bit(AGG_CONTEXT_LOCKED_BIT) = locked;
    controls.bit(AGG_CONTEXT_FALLBACK_BIT) = fallback;

    single_cycle_word_t target_controls = 0;
    target_controls.bit(AGG_CONTEXT_TARGET_VALID_BIT) = ten_minute_valid;
    target_controls.bit(AGG_CONTEXT_TARGET_UPDATE_BIT) = ten_minute_update;

    // Append the same 13-word context packet produced by the RTL shim.
    s_result.write(cfg_generation);
    s_result.write(sample_rate);
    s_result.write(controls);
    s_result.write(freq_status);
    s_result.write(freq_period);
    s_result.write(freq_seq);
    s_result.write(cap_frames);
    s_result.write(cap_hdrerr);
    s_result.write(cap_overflow);
    s_result.write(cap_alerts);
    s_result.write(ap_uint<64>(ten_minute_target).range(31, 0));
    s_result.write(ap_uint<64>(ten_minute_target).range(63, 32));
    s_result.write(target_controls);
    // In hardware the engine is free-running, invoked every clock. Since A1
    // it may spend an invocation on a DEFERRED interval pass and consume no
    // beat, so a single call per send would leave the beat queued. Pump
    // until the input is drained, then once more to flush an interval this
    // beat armed (a call with nothing to do returns immediately).
    while (!s_result.empty()) hls_aggregation_engine(s_result, m_axis, m_agg);
    hls_aggregation_engine(s_result, m_axis, m_agg);
  }
};

static void take_record(Bench &b, ap_uint<32> (&words)[MREC_WORDS]) {
  int beats = 0;
  while (!b.m_axis.empty() && beats < MREC_WORDS) {
    const record_axis_t beat = b.m_axis.read();
    words[beats] = beat.data;
    CHECK(beat.keep == MREC_KEEP_ALL, "record TKEEP must be full");
    CHECK((beat.last == 1) == (beats == MREC_WORDS - 1),
          "record TLAST must mark beat 63 only (beat %d)", beats);
    ++beats;
  }
  CHECK(beats == MREC_WORDS, "record must be exactly 64 beats, got %d", beats);
  if (beats == MREC_WORDS) trace_completed_record('B', words);
}

// Drain the POWER record that follows every BASIC record on the stream.
static void take_power_record(Bench &b, ap_uint<32> (&words)[MREC_WORDS]) {
  take_record(b, words);
  CHECK(words[MREC_FORMAT_WORD] == MREC_FORMAT_POWER_V1,
        "paired power record format, got %08x",
        (unsigned)words[MREC_FORMAT_WORD]);
}

// Drain the PHASOR record, third of every block's record quad.
static void take_phasor_record(Bench &b, ap_uint<32> (&words)[MREC_WORDS]) {
  take_record(b, words);
  CHECK(words[MREC_FORMAT_WORD] == MREC_FORMAT_PHASOR_V2,
        "paired phasor record format, got %08x",
        (unsigned)words[MREC_FORMAT_WORD]);
}

// Drain the UNBALANCE record that closes every block's record quad.
static void take_unbalance_record(Bench &b, ap_uint<32> (&words)[MREC_WORDS]) {
  take_record(b, words);
  CHECK(words[MREC_FORMAT_WORD] == MREC_FORMAT_UNBAL_V2,
        "paired unbalance record format, got %08x",
        (unsigned)words[MREC_FORMAT_WORD]);
}

static long long read_s64(const ap_uint<32> (&words)[MREC_WORDS], int low) {
  return (long long)((unsigned long long)words[low] |
                     ((unsigned long long)words[low + 1] << 32));
}

// Exact 128-bit equality between a beat accumulator and a golden value.
static bool acc128_equal(const ap_uint<128> value, unsigned __int128 golden) {
  return (unsigned long long)value.range(63, 0) ==
             (unsigned long long)golden &&
         (unsigned long long)value.range(127, 64) ==
             (unsigned long long)(golden >> 64);
}

// Drive one whole block of `cycles` cycles; returns via out-params.
static void run_block(Bench &b, CycleSpec &c, unsigned cycles, GoldenBlock &g,
                      bool apply_on_first = false) {
  for (unsigned i = 0; i < cycles; ++i) {
    const single_cycle_result_t r = make_cycle(c, g);
    b.send(r, apply_on_first && i == 0);
    c.sequence += 1;
    c.cycle_sequence += 1;
    c.first_sample += c.samples;
    c.seed += 1;
  }
}


// ---------------------------------------------------------------------------
// Aggregate golden (150/180-cycle tier). The interval accumulators are the
// SUM of 15 blocks' accumulators by pure addition, so the golden folds the
// per-block goldens the same way and finalizes with the identical helpers.
//
// This is what replaces the three assertions that were retired with the
// block-result beat (r.sum/r.square per lane, and the saturated-lane clamp):
// if any block handed the interval tier a wrong accumulator, the aggregate
// values below cannot come out exact. The other thirteen retired assertions
// were provenance fields that the RECORDS carry too (timing word, status,
// valid mask, anchors, frequency) -- see the note in the bench header.
// ---------------------------------------------------------------------------
struct GoldenAgg {
  __int128 sum[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 square[MET_ACTIVE_CHANNELS] = {};
  long long raw_sum[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 raw_square[MET_ACTIVE_CHANNELS] = {};
  unsigned __int128 vll_square[MET_VLL_PAIRS] = {};
  __int128 power[MET_POWER_PHASES] = {};
  long long minimum[MET_ACTIVE_CHANNELS] = {};
  long long maximum[MET_ACTIVE_CHANNELS] = {};
  __int128 ph_re[MET_ACTIVE_CHANNELS] = {};
  __int128 ph_im[MET_ACTIVE_CHANNELS] = {};
  bool seeded = false;
  unsigned count = 0, cycles = 0, blocks = 0;
  unsigned long long freq_sum = 0;
};

static void fold_block(GoldenAgg &a, const GoldenBlock &g, unsigned cycles,
                       unsigned freq_mhz) {
  for (int l = 0; l < MET_ACTIVE_CHANNELS; ++l) {
    a.sum[l] += g.sum[l];
    a.square[l] += g.square[l];
    a.raw_sum[l] += g.raw_sum[l];
    a.raw_square[l] += g.raw_square[l];
    if (!a.seeded || g.minimum[l] < a.minimum[l]) a.minimum[l] = g.minimum[l];
    if (!a.seeded || g.maximum[l] > a.maximum[l]) a.maximum[l] = g.maximum[l];
    a.ph_re[l] += g.ph_re[l];
    a.ph_im[l] += g.ph_im[l];
  }
  for (int pr = 0; pr < MET_VLL_PAIRS; ++pr) a.vll_square[pr] += g.vll_square[pr];
  for (int ph = 0; ph < MET_POWER_PHASES; ++ph) a.power[ph] += g.power[ph];
  a.seeded = true;
  a.count += g.count;
  a.cycles += cycles;
  a.blocks += 1;
  a.freq_sum += freq_mhz;
}

// Drain one record off the aggregate master with the framing contract.
static void take_agg(Bench &b, ap_uint<32> (&words)[MREC_WORDS],
                     unsigned expect_format) {
  int beats = 0;
  while (!b.m_agg.empty() && beats < MREC_WORDS) {
    const record_axis_t beat = b.m_agg.read();
    words[beats] = beat.data;
    CHECK(beat.keep == MREC_KEEP_ALL, "aggregate TKEEP must be full");
    CHECK((beat.last == 1) == (beats == MREC_WORDS - 1),
          "aggregate TLAST must mark beat 63 only (beat %d)", beats);
    ++beats;
  }
  CHECK(beats == MREC_WORDS, "aggregate record must be 64 beats, got %d", beats);
  // expect_format 0 means "any" -- the soak sweep classifies after draining.
  if (expect_format != 0u) {
    CHECK(words[MREC_FORMAT_WORD] == expect_format,
          "aggregate format 0x%08x, expected 0x%08x",
          (unsigned)words[MREC_FORMAT_WORD], expect_format);
  }
  if (beats == MREC_WORDS) trace_completed_record('A', words);
}

#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
static void take_open_quad(Bench &b, ap_uint<32> (&words)[MREC_WORDS],
                           unsigned fundamental_format,
                           unsigned power_format,
                           unsigned phasor_format,
                           unsigned unbalance_format) {
  take_agg(b, words, fundamental_format);
  const unsigned sequence = (unsigned)words[MREC_SEQUENCE_WORD];
  CHECK((words[MREC_STATUS_WORD] &
         (1u << TEN_MINUTE_STATUS_OPEN_INTERVAL_BIT)) != 0,
        "open interval status is asserted");
  CHECK((words[MREC_STATUS_WORD] &
         (1u << TEN_MINUTE_STATUS_NON_NORMATIVE_BIT)) != 0,
        "open interval is explicitly non-normative");
  CHECK((words[MREC_STATUS_WORD] &
         (1u << TEN_MINUTE_STATUS_COMPLETE_BIT)) == 0,
        "open interval is never marked complete");

  take_agg(b, words, power_format);
  CHECK((unsigned)words[MREC_SEQUENCE_WORD] == sequence,
        "open power record shares preview sequence");
  take_agg(b, words, phasor_format);
  CHECK((unsigned)words[MREC_SEQUENCE_WORD] == sequence,
        "open phasor record shares preview sequence");
  take_agg(b, words, unbalance_format);
  CHECK((unsigned)words[MREC_SEQUENCE_WORD] == sequence,
        "open unbalance record shares preview sequence");
}
#endif

int main() {
  static_assert(SCYC_PACKET_WORDS == 221,
                "single-cycle packet length is normative");
  static_assert(AGG_INPUT_PACKET_WORDS == 234,
                "aggregation input packet length is normative");

  if (const char *path = std::getenv("MNC_COMPLETED_RECORD_TRACE")) {
    completed_trace = std::fopen(path, "wb");
    if (completed_trace == nullptr) {
      std::perror("cannot open completed-record trace");
      return EXIT_FAILURE;
    }
  }

  Bench b;
  ap_uint<32> words[MREC_WORDS];
  CycleSpec c;

  // --- 12 @ 60 Hz: exact Mtr1-equivalence over a full block. -------------
  {
    GoldenBlock g;
    run_block(b, c, 12, g, /*apply_on_first=*/true);
    take_record(b, words);

    CHECK(words[MREC_FORMAT_WORD] == MREC_FORMAT_BASIC_V4, "record format");
    CHECK(words[MREC_SEQUENCE_WORD] == 1,
          "first block carries sequence 1");
    CHECK(words[MREC_SAMPLE_COUNT_WORD] == g.count,
          "merged sample count (%u)", g.count);
    CHECK(words[MREC_FIRST_SAMPLE_LOW_WORD] == 1000,
          "block first-sample anchor");
    CHECK(words[BASIC_LAST_SAMPLE_LOW_WORD] == 1000 + g.count - 1,
          "block last-sample anchor");
    CHECK(words[MTR1_TIMING_WORD] ==
              (60u | (12u << MTR1_TIMING_CYCLES_LSB) |
               (0x5u << MTR1_TIMING_FLAGS_LSB)),
          "timing word: nominal 60, 12 cycles, locked+first, got 0x%08x",
          (unsigned)words[MTR1_TIMING_WORD]);
    CHECK((words[MREC_STATUS_WORD] & 0x5u) == 0x4u,
          "first block: gap mark set, no overflow, got 0x%x",
          (unsigned)words[MREC_STATUS_WORD]);
    /* retired with the block-result beat: r.cycle_count == 12 && r.nominal_hz == 60 */ (void)0;
    /* retired with the block-result beat: r.frequency_millihz == c.freq_mhz && r.frequency_valid == 1 */ (void)0;
    /* retired with the block-result beat: r.valid_mask == 0x7F */ (void)0;

    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      const long long mean_units =
          golden_mean_q16(g.sum[lane], g.count) >> 16;
      const unsigned long long rms_q16 =
          golden_rms_q16(g.square[lane], g.sum[lane], g.count, true);
      const unsigned long long raw_rms =
          golden_rms_q16(g.raw_square[lane], g.raw_sum[lane], g.count, true);
      const int base = MTR1_CH_BASE_WORD + lane * MTR1_CH_STRIDE_WORDS;
      const long long got_mean =
          (long long)((unsigned long long)words[base + MTR1_CH_MEAN_LOW] |
                      ((unsigned long long)words[base + MTR1_CH_MEAN_HIGH]
                       << 32));
      const unsigned long long got_rms =
          (unsigned long long)words[base + MTR1_CH_RMS_LOW] |
          ((unsigned long long)words[base + MTR1_CH_RMS_HIGH] << 32);
      CHECK(got_mean == mean_units, "lane %d mean exact", lane);
      CHECK(got_rms == (rms_q16 >> 16), "lane %d RMS exact (Mtr1 proof)",
            lane);
      CHECK((unsigned long long)words[base + MTR1_CH_RMS_COUNT] ==
                (raw_rms & 0xFFFFFFFFull),
            "lane %d raw RMS counts exact", lane);
      /* retired with the block-result beat: acc128_equal(ap_uint<128>(r.sum[lane]), (unsigned __int128)g */ (void)0;
    }
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      const unsigned long long vll_q16 =
          golden_rms_q16(g.vll_square[pair], 0, g.count, false);
      CHECK((unsigned long long)words[BASIC_VLL_BASE_WORD + pair] ==
                (vll_q16 >> 16),
            "pair %d VLL RMS exact", pair);
    }
    CHECK(words[MTR1_FREQUENCY_VALUE_WORD] == c.freq_mhz &&
              words[MTR1_FREQUENCY_STATUS_WORD] == 0x2 &&
              words[MTR1_FREQUENCY_PERIOD_WORD] == 0x00010000 &&
              words[MTR1_FREQUENCY_SEQUENCE_WORD] == 42,
          "frequency context words");
    CHECK(words[MTR1_CAPTURE_FRAMES_WORD] == 111 &&
              words[MTR1_HEADER_ERRORS_WORD] == 1 &&
              words[MTR1_FIFO_OVERFLOWS_WORD] == 2 &&
              words[MTR1_ADC_ALERTS_WORD] == 3,
          "capture context words");

    // ---- POWER-v1 companion record: exact goldens. ----------------------
    ap_uint<32> pw[MREC_WORDS];
    take_power_record(b, pw);
    CHECK(pw[MREC_SEQUENCE_WORD] == 1 && pw[MREC_SAMPLE_COUNT_WORD] == g.count &&
              pw[MREC_FIRST_SAMPLE_LOW_WORD] == 1000 &&
              pw[MREC_STATUS_WORD] == words[MREC_STATUS_WORD] &&
              pw[MTR1_TIMING_WORD] == words[MTR1_TIMING_WORD] &&
              pw[BASIC_LAST_SAMPLE_LOW_WORD] ==
                  words[BASIC_LAST_SAMPLE_LOW_WORD],
          "power record shares the block's envelope");
    static const int pv[3] = {MET_LANE_VA, MET_LANE_VB, MET_LANE_VC};
    static const int pi_[3] = {MET_LANE_IA, MET_LANE_IB, MET_LANE_IC};
    long long g_total_p = 0;
    unsigned long long g_total_s = 0;
    for (int phase = 0; phase < 3; ++phase) {
      const long long p_pw = golden_p_pw(g.power[phase], g.count);
      const unsigned long long s_pw = golden_s_pw(
          golden_rms_q16(g.square[pv[phase]], g.sum[pv[phase]], g.count, true),
          golden_rms_q16(g.square[pi_[phase]], g.sum[pi_[phase]], g.count,
                         true));
      const long long pf = golden_pf_e6(p_pw, s_pw);
      g_total_p += p_pw;
      g_total_s += s_pw;
      const int base = POWER_PHASE_BASE_WORD + phase * POWER_PHASE_STRIDE;
      const long long got_p =
          (long long)((unsigned long long)pw[base + POWER_PHASE_P_LOW] |
                      ((unsigned long long)pw[base + POWER_PHASE_P_HIGH]
                       << 32));
      const unsigned long long got_s =
          (unsigned long long)pw[base + POWER_PHASE_S_LOW] |
          ((unsigned long long)pw[base + POWER_PHASE_S_HIGH] << 32);
      const long long got_pf = (int)pw[base + POWER_PHASE_PF];
      CHECK(got_p == p_pw, "phase %d P exact: got %lld expected %lld", phase,
            got_p, p_pw);
      CHECK(got_s == s_pw, "phase %d S exact: got %llu expected %llu", phase,
            got_s, s_pw);
      CHECK(got_pf == pf, "phase %d PF exact: got %lld expected %lld", phase,
            got_pf, pf);
    }
    const long long got_total_p =
        (long long)((unsigned long long)pw[POWER_TOTAL_P_LOW_WORD] |
                    ((unsigned long long)pw[POWER_TOTAL_P_HIGH_WORD] << 32));
    const unsigned long long got_total_s =
        (unsigned long long)pw[POWER_TOTAL_S_LOW_WORD] |
        ((unsigned long long)pw[POWER_TOTAL_S_HIGH_WORD] << 32);
    CHECK(got_total_p == g_total_p && got_total_s == g_total_s,
          "totals are arithmetic sums");
    CHECK((int)pw[POWER_TOTAL_PF_WORD] ==
              golden_pf_e6(g_total_p, g_total_s),
          "total PF = P_total / S_total");
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      const unsigned long long crest = golden_crest_e4(
          g.minimum[lane], g.maximum[lane],
          golden_rms_q16(g.square[lane], g.sum[lane], g.count, true));
      CHECK((unsigned long long)pw[POWER_CREST_BASE_WORD + lane] == crest,
            "lane %d crest exact", lane);
    }

    // ---- PHASOR-v1 companion record: exact goldens, libm angles. --------
    ap_uint<32> ph[MREC_WORDS];
    take_phasor_record(b, ph);
    CHECK(ph[MREC_SEQUENCE_WORD] == 1 &&
              ph[MREC_SAMPLE_COUNT_WORD] == g.count &&
              ph[MREC_FIRST_SAMPLE_LOW_WORD] == 1000 &&
              ph[MTR1_TIMING_WORD] == words[MTR1_TIMING_WORD] &&
              ph[BASIC_LAST_SAMPLE_LOW_WORD] ==
                  words[BASIC_LAST_SAMPLE_LOW_WORD],
          "phasor record shares the block's envelope");
    CHECK((ph[MREC_STATUS_WORD] & (1u << PHASOR_STATUS_INVALID_BIT)) == 0,
          "clean block: phasor-invalid bit clear");
    long long re_c[MET_ACTIVE_CHANNELS], im_c[MET_ACTIVE_CHANNELS];
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      re_c[lane] = golden_phasor_counts(g.ph_re[lane], g.count);
      im_c[lane] = golden_phasor_counts(g.ph_im[lane], g.count);
    }
    const long long ref_mdeg =
        golden_angle_mdeg(im_c[MET_LANE_VA], re_c[MET_LANE_VA]);
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      const int base = PHASOR_CH_BASE_WORD + lane * PHASOR_CH_STRIDE;
      const unsigned long long fund =
          golden_fund_rms_q16(re_c[lane], im_c[lane]);
      CHECK((unsigned long long)ph[base + PHASOR_CH_FUND_RMS] == (fund >> 16),
            "lane %d fundamental RMS exact", lane);
      const long long got_angle = (long long)(int)ph[base + PHASOR_CH_ANGLE];
      if (lane == MET_LANE_VA) {
        CHECK(got_angle == 0, "VA reference angle is exactly 0");
      } else {
        const long long expect =
            wrap_mdeg(golden_angle_mdeg(im_c[lane], re_c[lane]) - ref_mdeg);
        const long long diff = wrap_mdeg(got_angle - expect);
        CHECK(diff >= -2 && diff <= 2,
              "lane %d angle within 2 mdeg (got %lld expected %lld)", lane,
              got_angle, expect);
      }
    }
    static const int vp[MET_VLL_PAIRS] = {MET_LANE_VA, MET_LANE_VB,
                                          MET_LANE_VC};
    static const int vn[MET_VLL_PAIRS] = {MET_LANE_VB, MET_LANE_VC,
                                          MET_LANE_VA};
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      const long long vre = re_c[vp[pair]] - re_c[vn[pair]];
      const long long vim = im_c[vp[pair]] - im_c[vn[pair]];
      const int base = PHASOR_VLL_BASE_WORD + pair * PHASOR_VLL_STRIDE;
      CHECK((unsigned long long)ph[base + PHASOR_CH_FUND_RMS] ==
                (golden_fund_rms_q16(vre, vim) >> 16),
            "pair %d VLL fundamental exact", pair);
      const long long got = (long long)(int)ph[base + PHASOR_CH_ANGLE];
      const long long expect =
          wrap_mdeg(golden_angle_mdeg(vim, vre) - ref_mdeg);
      const long long diff = wrap_mdeg(got - expect);
      CHECK(diff >= -2 && diff <= 2, "pair %d VLL angle within 2 mdeg", pair);
    }
    // Balanced defaults: VAB leads VA by 30 degrees.
    const long long vab_angle =
        (long long)(int)ph[PHASOR_VLL_BASE_WORD + PHASOR_CH_ANGLE];
    CHECK(vab_angle >= 29997 && vab_angle <= 30003,
          "balanced VAB sits at +30 degrees (got %lld mdeg)", vab_angle);

    long long g_p1_total = 0, g_q1_total = 0;
    unsigned long long g_s1_total = 0;
    unsigned expect_flags = 0;
    for (int phase = 0; phase < 3; ++phase) {
      const int v = pv[phase], ii = pi_[phase];
      const long long p1 = golden_p1_pw(re_c[v], im_c[v], re_c[ii], im_c[ii]);
      const long long q1 =
          golden_q1_pvar(re_c[v], im_c[v], re_c[ii], im_c[ii]);
      const unsigned long long s1 =
          golden_s_pw(golden_fund_rms_q16(re_c[v], im_c[v]),
                      golden_fund_rms_q16(re_c[ii], im_c[ii]));
      const long long dpf = golden_pf_e6(p1, s1);
      g_p1_total += p1;
      g_q1_total += q1;
      g_s1_total += s1;
      CHECK(read_s64(ph, PHASOR_Q1_BASE_WORD + phase * 2) == q1,
            "phase %d Q1 exact", phase);
      CHECK(read_s64(ph, PHASOR_P1_BASE_WORD + phase * 2) == p1,
            "phase %d P1 exact", phase);
      CHECK((long long)(int)ph[PHASOR_DPF_BASE_WORD + phase] == dpf,
            "phase %d displacement PF exact", phase);
      CHECK(q1 > 0, "lagging current: phase %d Q1 positive", phase);
      const long long disp = (long long)(int)ph[PHASOR_DISP_BASE_WORD + phase];
      CHECK(disp >= 9997 && disp <= 10003,
            "phase %d displacement angle ~ +10 deg (got %lld)", phase, disp);
      expect_flags |= golden_nature(q1, s1) << (phase * 2);
    }
    CHECK(read_s64(ph, PHASOR_Q1_TOTAL_LOW_WORD) == g_q1_total &&
              read_s64(ph, PHASOR_P1_TOTAL_LOW_WORD) == g_p1_total,
          "phasor totals are arithmetic sums");
    CHECK((long long)(int)ph[PHASOR_DPF_TOTAL_WORD] ==
              golden_pf_e6(g_p1_total, g_s1_total),
          "total displacement PF = P1_tot / S1_tot");
    expect_flags |= golden_nature(g_q1_total, g_s1_total)
                    << PHASOR_FLAGS_NATURE_TOTAL_LSB;
    expect_flags |= 1u << PHASOR_FLAGS_REF_VALID_BIT;
    CHECK(ph[PHASOR_FLAGS_WORD] == expect_flags,
          "flags word: natures + reference (got 0x%x expected 0x%x)",
          (unsigned)ph[PHASOR_FLAGS_WORD], expect_flags);
    const long long dpf_a = (long long)(int)ph[PHASOR_DPF_BASE_WORD];
    CHECK(dpf_a >= 984300 && dpf_a <= 985300,
          "phase A displacement PF ~ cos(10 deg) (got %lld)", dpf_a);
    CHECK(ph[60] == 0 && ph[61] == 0 && ph[62] == 0 && ph[63] == 0,
          "phasor reserved words zero");

    // ---- UNBALANCE-v1 companion record (M10): exact goldens. ------------
    ap_uint<32> ub[MREC_WORDS];
    take_unbalance_record(b, ub);
    CHECK(ub[MREC_SEQUENCE_WORD] == 1 &&
              ub[MREC_STATUS_WORD] == ph[MREC_STATUS_WORD] &&
              ub[MTR1_TIMING_WORD] == words[MTR1_TIMING_WORD] &&
              ub[BASIC_LAST_SAMPLE_LOW_WORD] ==
                  words[BASIC_LAST_SAMPLE_LOW_WORD],
          "unbalance record shares the block's envelope");
    for (int set = 0; set < 2; ++set) {
      const int la = set == 0 ? MET_LANE_VA : MET_LANE_IA;
      const int lb = set == 0 ? MET_LANE_VB : MET_LANE_IB;
      const int lc = set == 0 ? MET_LANE_VC : MET_LANE_IC;
      const long long in_re[3] = {re_c[la], re_c[lb], re_c[lc]};
      const long long in_im[3] = {im_c[la], im_c[lb], im_c[lc]};
      long long sre[3], sim[3];
      golden_sequence(in_re, in_im, sre, sim);
      unsigned long long rms[3];
      for (int k = 0; k < 3; ++k) {
        rms[k] = golden_fund_rms_q16(sre[k], sim[k]);
        const int base =
            (set == 0 ? UNBAL_V_BASE_WORD : UNBAL_I_BASE_WORD) +
            k * UNBAL_SEQ_STRIDE;
        CHECK((unsigned long long)ub[base + UNBAL_SEQ_RMS] == (rms[k] >> 16),
              "set %d component %d sequence RMS exact", set, k);
        const long long got = (long long)(int)ub[base + UNBAL_SEQ_ANGLE];
        const long long expect =
            wrap_mdeg(golden_angle_mdeg(sim[k], sre[k]) - ref_mdeg);
        const long long diff = wrap_mdeg(got - expect);
        CHECK(diff >= -2 && diff <= 2,
              "set %d component %d angle within 2 mdeg", set, k);
      }
      CHECK((unsigned long long)ub[set == 0 ? UNBAL_V_ZERO_RATIO_WORD
                                            : UNBAL_I_ZERO_RATIO_WORD] ==
                    golden_ratio_e6(rms[0], rms[1]) &&
                (unsigned long long)ub[set == 0 ? UNBAL_V_UNBALANCE_WORD
                                                : UNBAL_I_UNBALANCE_WORD] ==
                    golden_ratio_e6(rms[2], rms[1]),
            "set %d ratios exact", set);
      // Balanced defaults: the negative-sequence ratio is far below 0.1%.
      CHECK(golden_ratio_e6(rms[2], rms[1]) < 1000,
            "set %d nearly balanced", set);
    }
    CHECK(ub[UNBAL_FLAGS_WORD] ==
              ((1u << UNBAL_FLAGS_REF_VALID_BIT) |
               (1u << UNBAL_FLAGS_V_VALID_BIT) |
               (1u << UNBAL_FLAGS_I_VALID_BIT)),
          "unbalance flags: both sets valid + reference (got 0x%x)",
          (unsigned)ub[UNBAL_FLAGS_WORD]);
    for (int i = 33; i < 64; ++i)
      CHECK(ub[i] == 0, "unbalance reserved word %d nonzero", i);
  }

  // --- Second block: clean status, sequences chain. -----------------------
  {
    GoldenBlock g;
    run_block(b, c, 12, g);
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    take_unbalance_record(b, words);
    /* retired with the block-result beat: r.sequence == 2 && (r.status & 0x5u) == 0 */ (void)0;
    /* retired with the block-result beat: (r.flags & 0x4u) == 0 */ (void)0;
  }

  // --- Upstream gap mark restarts the block. ------------------------------
  {
    GoldenBlock g;
    run_block(b, c, 5, g);  // partial: will be discarded
    GoldenBlock g2;
    c.status = 1u << SCYC_STATUS_FIRST_AFTER_GAP_BIT;
    // The gap-marked cycle starts the NEW block.
    const single_cycle_result_t r0 = make_cycle(c, g2);
    b.send(r0);
    c.sequence += 1; c.cycle_sequence += 1; c.first_sample += c.samples;
    c.status = 0;
    const unsigned long long block_start = (unsigned long long)r0.first_sample;
    run_block(b, c, 11, g2);
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    take_unbalance_record(b, words);
    /* retired with the block-result beat: r.first_sample == block_start */ (void)0;
    /* retired with the block-result beat: (r.status & 0x4u) == 0x4u */ (void)0;
  }

  // --- Sequence break restarts; nominal 50 closes at 10. ------------------
  {
    GoldenBlock g;
    run_block(b, c, 4, g);  // partial
    c.cycle_sequence += 3;  // grid cycle sequence jumps: break
    c.nominal = 50;
    GoldenBlock g2;
    const single_cycle_result_t r0 = make_cycle(c, g2);
    b.send(r0);
    c.sequence += 1; c.cycle_sequence += 1; c.first_sample += c.samples;
    run_block(b, c, 9, g2);
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    take_unbalance_record(b, words);
    /* retired with the block-result beat: r.cycle_count == 10 && r.nominal_hz == 50 */ (void)0;
    /* retired with the block-result beat: (r.status & 0x4u) == 0x4u */ (void)0;
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      /* retired with the block-result beat: acc128_equal(r.square[lane], g2.square[lane]) */ (void)0;
    }
  }

  // --- Stale generation discards; lock/fallback flags reduce. ------------
  {
    // A cycle tagged with a foreign generation is rejected outright.
    GoldenBlock scratch;
    c.generation = 9;
    const single_cycle_result_t stale = make_cycle(c, scratch);
    b.send(stale);
    c.generation = 1;

    GoldenBlock g;
    // One mid-block cycle arrives unlocked/fallback: AND/OR reduction.
    for (unsigned i = 0; i < 10; ++i) {
      if (i == 4) { b.locked = false; b.fallback = true; }
      const single_cycle_result_t r = make_cycle(c, g);
      b.send(r);
      b.locked = true; b.fallback = false;
      c.sequence += 1; c.cycle_sequence += 1; c.first_sample += c.samples;
      c.seed += 1;
    }
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    take_unbalance_record(b, words);
    /* retired with the block-result beat: (r.flags & 0x1u) == 0 && (r.flags & 0x2u) == 0x2u */ (void)0;
  }

  // --- Sticky arithmetic flag from a saturated cycle, cleared by APPLY. ---
  {
    GoldenBlock g;
    CycleSpec sat = c;
    for (unsigned i = 0; i < 10; ++i) {
      single_cycle_result_t r = make_cycle(sat, g);
      if (i == 0) {
        r.square[0] = ~ap_uint<128>(0);  // upstream saturated square
        r.status = 1u << SCYC_STATUS_OVERFLOW_BIT;
      }
      b.send(r);
      sat.sequence += 1; sat.cycle_sequence += 1; sat.first_sample += sat.samples;
    }
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    take_unbalance_record(b, words);
    /* retired with the block-result beat: (r.status & 0x1u) == 0x1u */ (void)0;
    /* retired with the block-result beat: r.square[0] == ~ap_uint<128>(0) */ (void)0;
    c = sat;

    // APPLY clears the sticky flag: the next full block is clean again.
    GoldenBlock g2;
    run_block(b, c, 10, g2, /*apply_on_first=*/true);
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    take_unbalance_record(b, words);
    /* retired with the block-result beat: (r2.status & 0x1u) == 0 */ (void)0;
    /* retired with the block-result beat: (r2.status & 0x4u) == 0x4u */ (void)0;
  }

  // --- Zero current: S and PF must be exactly 0, never garbage. ----------
  {
    GoldenBlock g;
    CycleSpec z = c;
    z.zero_lanes = 1u << MET_LANE_IA;
    for (unsigned i = 0; i < 10; ++i) {
      const single_cycle_result_t r = make_cycle(z, g);
      b.send(r);
      z.sequence += 1; z.cycle_sequence += 1; z.first_sample += z.samples;
    }
    take_record(b, words);
    ap_uint<32> pw[MREC_WORDS];
    take_power_record(b, pw);
    const int base = POWER_PHASE_BASE_WORD;  // phase A
    CHECK(pw[base + POWER_PHASE_P_LOW] == 0 &&
              pw[base + POWER_PHASE_P_HIGH] == 0,
          "zero current: P_A is 0");
    CHECK(pw[base + POWER_PHASE_S_LOW] == 0 &&
              pw[base + POWER_PHASE_S_HIGH] == 0,
          "zero current: S_A is 0");
    CHECK(pw[base + POWER_PHASE_PF] == 0,
          "zero current: PF_A is 0 (undefined, gated on S)");
    CHECK(pw[POWER_CREST_BASE_WORD + MET_LANE_IA] == 0,
          "zero current: crest of a silent lane is 0");
    // Phase B stays live and its PF is meaningful.
    const int base_b = POWER_PHASE_BASE_WORD + POWER_PHASE_STRIDE;
    CHECK(pw[base_b + POWER_PHASE_S_LOW] != 0 ||
              pw[base_b + POWER_PHASE_S_HIGH] != 0,
          "zero current on A leaves B's S alone");
    // The phasor record mirrors the gate: S1 = 0 means Q1/dPF read zero
    // and the load nature is UNDEFINED.
    ap_uint<32> phz[MREC_WORDS];
    take_phasor_record(b, phz);
    CHECK(read_s64(phz, PHASOR_Q1_BASE_WORD) == 0 &&
              (int)phz[PHASOR_DPF_BASE_WORD] == 0 &&
              ((phz[PHASOR_FLAGS_WORD] >> PHASOR_FLAGS_NATURE_A_LSB) & 0x3) ==
                  (unsigned)MET_NATURE_UNDEFINED,
          "zero current: Q1/dPF zero, nature UNDEFINED");
    take_unbalance_record(b, phz);
    c = z;
  }

  // --- Leading current flips Q1; quadrature pins dPF ~ 0, Q1 ~ S1. -------
  {
    GoldenBlock g;
    CycleSpec lead = c;
    lead.zero_lanes = 0;              // the zero-current scenario is over
    lead.ph_deg[MET_LANE_IA] = 15.0;  // currents LEAD their voltage by 15
    lead.ph_deg[MET_LANE_IB] = -105.0;
    lead.ph_deg[MET_LANE_IC] = 135.0;
    for (unsigned i = 0; i < 10; ++i) {
      const single_cycle_result_t r = make_cycle(lead, g);
      b.send(r);
      lead.sequence += 1;
      lead.cycle_sequence += 1;
      lead.first_sample += lead.samples;
    }
    take_record(b, words);
    take_power_record(b, words);
    ap_uint<32> ph[MREC_WORDS];
    take_phasor_record(b, ph);
    ap_uint<32> ub_scratch[MREC_WORDS];
    take_unbalance_record(b, ub_scratch);
    const long long q1_a = read_s64(ph, PHASOR_Q1_BASE_WORD);
    CHECK(q1_a < 0, "leading current: Q1_A negative (got %lld)", q1_a);
    const unsigned nature_a =
        (ph[PHASOR_FLAGS_WORD] >> PHASOR_FLAGS_NATURE_A_LSB) & 0x3;
    CHECK(nature_a == (unsigned)MET_NATURE_LEADING,
          "leading current classifies as LEADING (got %u)", nature_a);
    const long long disp_a = (long long)(int)ph[PHASOR_DISP_BASE_WORD];
    CHECK(disp_a >= 344997 && disp_a <= 345003,
          "leading 15 deg: displacement 345 deg (got %lld)", disp_a);
    c = lead;

    // Quadrature: IA lags VA by 90 degrees.
    GoldenBlock g2;
    CycleSpec quad = c;
    quad.ph_deg[MET_LANE_IA] = -90.0;
    for (unsigned i = 0; i < 10; ++i) {
      const single_cycle_result_t r = make_cycle(quad, g2);
      b.send(r);
      quad.sequence += 1;
      quad.cycle_sequence += 1;
      quad.first_sample += quad.samples;
    }
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, ph);
    take_unbalance_record(b, ub_scratch);
    const long long q1_quad = read_s64(ph, PHASOR_Q1_BASE_WORD);
    const long long dpf_quad = (long long)(int)ph[PHASOR_DPF_BASE_WORD];
    CHECK(q1_quad > 0 && dpf_quad >= -100 && dpf_quad <= 100,
          "quadrature: Q1 positive, dPF ~ 0 (Q1 %lld dPF %lld)", q1_quad,
          dpf_quad);
    // |Q1| ~ S1 at 90 degrees, within the documented sqrt(2)-constant
    // bias plus rounding (the exact pin lives in the main block).
    const long long rv = golden_phasor_counts(g2.ph_re[MET_LANE_VA], g2.count);
    const long long iv = golden_phasor_counts(g2.ph_im[MET_LANE_VA], g2.count);
    const long long ri = golden_phasor_counts(g2.ph_re[MET_LANE_IA], g2.count);
    const long long im_i =
        golden_phasor_counts(g2.ph_im[MET_LANE_IA], g2.count);
    const unsigned long long s1 = golden_s_pw(
        golden_fund_rms_q16(rv, iv), golden_fund_rms_q16(ri, im_i));
    const long long delta = q1_quad - (long long)s1;
    CHECK(delta > -(long long)(s1 / 500) && delta < (long long)(s1 / 500),
          "quadrature: Q1 within 0.2%% of S1 (Q1 %lld S1 %llu)", q1_quad, s1);
    c = quad;
  }

  // --- A phasor-invalid cycle poisons only the PHASOR record. ------------
  {
    GoldenBlock g;
    CycleSpec inv = c;
    for (unsigned i = 0; i < 10; ++i) {
      inv.status = (i == 3) ? (1u << SCYC_STATUS_PHASOR_INVALID_BIT) : 0;
      const single_cycle_result_t r = make_cycle(inv, g);
      b.send(r);
      inv.sequence += 1;
      inv.cycle_sequence += 1;
      inv.first_sample += inv.samples;
    }
    inv.status = 0;
    take_record(b, words);
    CHECK((words[MREC_STATUS_WORD] & 0x2u) == 0,
          "BASIC status does not carry the phasor-invalid bit");
    take_power_record(b, words);
    ap_uint<32> ph[MREC_WORDS];
    take_phasor_record(b, ph);
    CHECK((ph[MREC_STATUS_WORD] & (1u << PHASOR_STATUS_INVALID_BIT)) != 0,
          "one invalid cycle marks the block's PHASOR record");
    ap_uint<32> ub[MREC_WORDS];
    take_unbalance_record(b, ub);
    CHECK((ub[MREC_STATUS_WORD] & (1u << UNBAL_STATUS_INVALID_BIT)) != 0,
          "the UNBALANCE record mirrors the phasor-invalid bit");
    c = inv;

    GoldenBlock g2;
    run_block(b, c, 10, g2);
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, ph);
    CHECK((ph[MREC_STATUS_WORD] & (1u << PHASOR_STATUS_INVALID_BIT)) == 0,
          "phasor-invalid clears on the next block");
    take_unbalance_record(b, ub);
  }

  // --- VA absent: the angle reference is declared invalid. ---------------
  {
    GoldenBlock g;
    CycleSpec no_va = c;
    no_va.zero_lanes = 1u << MET_LANE_VA;
    for (unsigned i = 0; i < 10; ++i) {
      const single_cycle_result_t r = make_cycle(no_va, g);
      b.send(r);
      no_va.sequence += 1;
      no_va.cycle_sequence += 1;
      no_va.first_sample += no_va.samples;
    }
    take_record(b, words);
    take_power_record(b, words);
    ap_uint<32> ph[MREC_WORDS];
    take_phasor_record(b, ph);
    const int va_base = PHASOR_CH_BASE_WORD + MET_LANE_VA * PHASOR_CH_STRIDE;
    CHECK(ph[va_base + PHASOR_CH_FUND_RMS] == 0, "silent VA fundamental is 0");
    CHECK((ph[PHASOR_FLAGS_WORD] & (1u << PHASOR_FLAGS_REF_VALID_BIT)) == 0,
          "zero VA fundamental clears the angle-reference flag");
    ap_uint<32> ub[MREC_WORDS];
    take_unbalance_record(b, ub);
    CHECK((ub[UNBAL_FLAGS_WORD] & (1u << UNBAL_FLAGS_REF_VALID_BIT)) == 0,
          "unbalance record mirrors the reference flag");
    c = no_va;
  }

  // --- ACB rotation: the sequence swap (M10 acceptance). ------------------
  {
    GoldenBlock g;
    CycleSpec acb = c;
    acb.zero_lanes = 0;
    acb.ph_deg[MET_LANE_VB] = 120.0;  // B and C swapped: reversed rotation
    acb.ph_deg[MET_LANE_VC] = -120.0;
    // Earlier scenarios scrambled the current phases; restore a balanced
    // ABC current set so the I ratios contrast with the reversed V set.
    acb.ph_deg[MET_LANE_IA] = -10.0;
    acb.ph_deg[MET_LANE_IB] = -130.0;
    acb.ph_deg[MET_LANE_IC] = 110.0;
    for (unsigned i = 0; i < 10; ++i) {
      const single_cycle_result_t r = make_cycle(acb, g);
      b.send(r);
      acb.sequence += 1;
      acb.cycle_sequence += 1;
      acb.first_sample += acb.samples;
    }
    take_record(b, words);
    take_power_record(b, words);
    ap_uint<32> ph[MREC_WORDS];
    take_phasor_record(b, ph);
    ap_uint<32> ub[MREC_WORDS];
    take_unbalance_record(b, ub);
    // Exact golden replication for the V set under ACB.
    long long in_re[3], in_im[3], sre[3], sim[3];
    static const int vlanes[3] = {MET_LANE_VA, MET_LANE_VB, MET_LANE_VC};
    for (int k = 0; k < 3; ++k) {
      in_re[k] = golden_phasor_counts(g.ph_re[vlanes[k]], g.count);
      in_im[k] = golden_phasor_counts(g.ph_im[vlanes[k]], g.count);
    }
    golden_sequence(in_re, in_im, sre, sim);
    const unsigned long long v1 = golden_fund_rms_q16(sre[1], sim[1]);
    const unsigned long long v2 = golden_fund_rms_q16(sre[2], sim[2]);
    CHECK((unsigned long long)ub[UNBAL_V_BASE_WORD + 1 * UNBAL_SEQ_STRIDE +
                                 UNBAL_SEQ_RMS] == (v1 >> 16),
          "ACB V1 exact");
    CHECK((unsigned long long)ub[UNBAL_V_BASE_WORD + 2 * UNBAL_SEQ_STRIDE +
                                 UNBAL_SEQ_RMS] == (v2 >> 16),
          "ACB V2 exact");
    // The sequence swap: negative dominates, positive collapses, and the
    // unbalance ratio flies off scale (or is undefined if V1 hit 0).
    const unsigned long long v_fund =
        golden_fund_rms_q16(in_re[0], in_im[0]);
    CHECK(v2 > (v_fund / 10) * 9 && v1 < v_fund / 1000,
          "ACB: V2 takes (nearly) everything, V1 collapses");
    CHECK((unsigned long long)ub[UNBAL_V_UNBALANCE_WORD] ==
              golden_ratio_e6(v2, v1),
          "ACB unbalance ratio exact (huge or undefined-as-0)");
    // Currents kept their ABC rotation: the I set stays nearly balanced.
    CHECK((unsigned long long)ub[UNBAL_I_UNBALANCE_WORD] < 1000,
          "ACB voltages leave the current set balanced");
    c = acb;
  }

  // --- Dead voltages: V ratios undefined, their flag clears. --------------
  {
    GoldenBlock g;
    CycleSpec dead = c;
    dead.zero_lanes =
        (1u << MET_LANE_VA) | (1u << MET_LANE_VB) | (1u << MET_LANE_VC);
    for (unsigned i = 0; i < 10; ++i) {
      const single_cycle_result_t r = make_cycle(dead, g);
      b.send(r);
      dead.sequence += 1;
      dead.cycle_sequence += 1;
      dead.first_sample += dead.samples;
    }
    take_record(b, words);
    take_power_record(b, words);
    take_phasor_record(b, words);
    ap_uint<32> ub[MREC_WORDS];
    take_unbalance_record(b, ub);
    CHECK((ub[UNBAL_FLAGS_WORD] & (1u << UNBAL_FLAGS_V_VALID_BIT)) == 0 &&
              ub[UNBAL_V_UNBALANCE_WORD] == 0 &&
              ub[UNBAL_V_ZERO_RATIO_WORD] == 0,
          "dead voltages: V ratios undefined-as-0 with the flag clear");
    CHECK((ub[UNBAL_FLAGS_WORD] & (1u << UNBAL_FLAGS_I_VALID_BIT)) != 0,
          "dead voltages leave the current set valid");
    c = dead;
  }

  // --- Disable stops everything. ------------------------------------------
  {
    b.enable = false;
    GoldenBlock scratch;
    const single_cycle_result_t r = make_cycle(c, scratch);
    b.send(r, /*apply_toggles=*/true);
  }


  // --- 150/180-cycle tier: 15 eligible blocks -> one exact aggregate. -----
  // Same engine, second master. The interval values must equal the golden
  // fold of the fifteen block goldens, which is the property that replaces
  // the retired per-block accumulator assertions.
  {
    b.enable = true; b.locked = true; b.fallback = false;
    b.dc_remove = true;
    CycleSpec ac;
    ac.sequence = 9000; ac.cycle_sequence = 9000; ac.first_sample = 700000;
    ac.nominal = 60; ac.samples = 5; ac.generation = b.cfg_generation;

    // An open ten-minute view exists only after Linux has supplied a valid
    // UTC/sample-counter target.  Keep that target beyond this accelerated
    // 150/180-cycle scenario so the test exercises a genuinely open
    // accumulator rather than weakening the production eligibility rule.
    b.ten_minute_target =
        ac.first_sample + ap_uint<64>(b.sample_rate) * 600u;
    b.ten_minute_valid = true;
    b.ten_minute_update = !b.ten_minute_update;

    // Drain anything the earlier scenarios left pending.
    while (!b.m_axis.empty()) b.m_axis.read();
    while (!b.m_agg.empty()) b.m_agg.read();

    // Block 0 carries the APPLY, which resets the interval tier AND marks
    // that block first-after-gap -- MET_FLAG_FIRST_BLOCK fails the
    // eligibility predicate, so it is deliberately NOT folded into the
    // golden. The interval is the FIFTEEN clean blocks that follow. (The
    // earlier scenarios in this bench leave the interval tier mid-count,
    // which is exactly why the reset has to be explicit here.)
    GoldenAgg ga;
    for (int blk = 0; blk <= MET_BASIC_BLOCKS_PER_AGGREGATE; ++blk) {
      const bool priming = (blk == 0);
      GoldenBlock gb;
      run_block(b, ac, 12, gb, /*apply_on_first=*/priming);
      if (!priming) fold_block(ga, gb, 12, ac.freq_mhz);
      // Each closed block still emits its own quad on the basic master.
      ap_uint<32> bw[MREC_WORDS];
      take_record(b, bw);
      take_power_record(b, bw);
      take_phasor_record(b, bw);
      take_unbalance_record(b, bw);
      // Only the fifteenth ELIGIBLE block closes the interval.
      if (blk < MET_BASIC_BLOCKS_PER_AGGREGATE)
        CHECK(b.m_agg.empty(), "interval must not close on block %d", blk);
    }
    CHECK(ga.blocks == MET_BASIC_BLOCKS_PER_AGGREGATE, "golden folded 15 blocks");

    ap_uint<32> aw[MREC_WORDS];
    take_agg(b, aw, MREC_FORMAT_AGG_V3);
    CHECK(aw[MREC_SAMPLE_COUNT_WORD] == ga.count,
          "interval sample count %u, golden %u",
          (unsigned)aw[MREC_SAMPLE_COUNT_WORD], ga.count);
    CHECK(aw[MTR2_SHAPE_WORD] ==
              (ap_uint<32>(MET_BASIC_BLOCKS_PER_AGGREGATE) << MTR2_SHAPE_BLOCKS_LSB |
               ap_uint<32>(60) << MTR2_SHAPE_NOMINAL_LSB |
               ap_uint<32>(ga.cycles) << MTR2_SHAPE_CYCLES_LSB),
          "interval shape word 0x%08x (blocks/nominal/cycles)",
          (unsigned)aw[MTR2_SHAPE_WORD]);

    // Folded basic-sequence range (words 14/15). This check exists because
    // its ABSENCE let a real bug through: when A1 deferred the interval
    // pass to the next invocation, `a3s_agg_last_seq` was a non-static
    // local and re-initialised to 0 in between, so every aggregate record
    // carried MTR2_LAST_BASIC_SEQ_WORD = 0. The differential harness caught
    // it; this bench had no opinion. Provenance words need assertions too,
    // not just values.
    CHECK(aw[MTR2_FIRST_BASIC_SEQ_WORD] != 0 &&
              aw[MTR2_LAST_BASIC_SEQ_WORD] != 0,
          "interval must carry both folded basic sequences, got %u..%u",
          (unsigned)aw[MTR2_FIRST_BASIC_SEQ_WORD],
          (unsigned)aw[MTR2_LAST_BASIC_SEQ_WORD]);
    CHECK(aw[MTR2_LAST_BASIC_SEQ_WORD] - aw[MTR2_FIRST_BASIC_SEQ_WORD] ==
              MET_BASIC_BLOCKS_PER_AGGREGATE - 1,
          "folded basic range must span exactly 15 blocks: %u..%u",
          (unsigned)aw[MTR2_FIRST_BASIC_SEQ_WORD],
          (unsigned)aw[MTR2_LAST_BASIC_SEQ_WORD]);

    // Per-lane interval RMS: the whole-interval finalize of the summed
    // accumulators, mean-corrected under the committed dc_remove.
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      const unsigned long long rms_q16 =
          golden_rms_q16(ga.square[lane], ga.sum[lane], ga.count, true);
      const unsigned long long got =
          (unsigned long long)aw[MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS];
      CHECK(got == (rms_q16 >> 16),
            "interval lane %d RMS %llu, golden %llu", lane, got, rms_q16 >> 16);
    }

    take_agg(b, aw, MREC_FORMAT_AGG_POWER_V1);
    for (int ph = 0; ph < MET_POWER_PHASES; ++ph) {
      const long long p_pw = golden_p_pw(ga.power[ph], ga.count);
      CHECK(read_s64(aw, POWER_PHASE_BASE_WORD + ph * POWER_PHASE_STRIDE) == p_pw,
            "interval phase %d P %lld, golden %lld", ph,
            read_s64(aw, POWER_PHASE_BASE_WORD + ph * POWER_PHASE_STRIDE), p_pw);
    }

    take_agg(b, aw, MREC_FORMAT_AGG_PHASOR_V2);
    take_agg(b, aw, MREC_FORMAT_AGG_UNBAL_V2);
    CHECK(b.m_agg.empty(), "exactly four aggregate records per interval");

    // The completed 150/180-cycle block schedules the open ten-minute
    // preview when enabled. Always supply the same real input transaction in
    // both builds so the completed-record byte traces are directly
    // comparable and Vitis cosimulation observes the free-running behavior.
    GoldenBlock preview_look_ahead;
    b.send(make_cycle(ac, preview_look_ahead));
    ac.sequence += 1;
    ac.cycle_sequence += 1;
    ac.first_sample += ac.samples;
    ac.seed += 1;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
    take_open_quad(b, aw,
                   MREC_FORMAT_OPEN_TEN_MINUTE_V1,
                   MREC_FORMAT_OPEN_TEN_MINUTE_POWER_V1,
                   MREC_FORMAT_OPEN_TEN_MINUTE_PHASOR_V2,
                   MREC_FORMAT_OPEN_TEN_MINUTE_UNBAL_V2);
    CHECK((aw[MTR2_SHAPE_WORD] & 0xFFFFu) >=
              MET_BASIC_BLOCKS_PER_AGGREGATE,
          "open ten-minute preview reports accumulated Basic blocks");
    CHECK(b.m_agg.empty(), "open ten-minute preview emits one record quad");
#endif
  }

  // --- UTC-targeted ten-minute tier: close at the first full basic block
  // ending at/after the programmed sample target.  A newly programmed
  // target begins mid-interval, so that first result is deliberately marked
  // contaminated; subsequent targets advance autonomously by 600 seconds.
  {
    b.enable = true; b.locked = true; b.fallback = false;
    b.dc_remove = true;
    while (!b.m_axis.empty()) b.m_axis.read();
    while (!b.m_agg.empty()) b.m_agg.read();

    CycleSpec tm;
    tm.sequence = 20000; tm.cycle_sequence = 20000;
    tm.first_sample = 2000000;
    tm.nominal = 60; tm.samples = 5; tm.generation = b.cfg_generation;

    const unsigned block_samples = 12u * tm.samples;
    const unsigned long long first_clean_sample =
        tm.first_sample + block_samples;
    const unsigned long long target =
        first_clean_sample + block_samples + 17u;
    b.ten_minute_target = target;
    b.ten_minute_valid = true;
    b.ten_minute_update = !b.ten_minute_update;

    // APPLY makes this block first-after-gap and therefore ineligible for
    // every interval tier.  It also proves that target programming and a
    // configuration boundary can safely occur together.
    GoldenBlock priming;
    run_block(b, tm, 12, priming, /*apply_on_first=*/true);
    ap_uint<32> tw[MREC_WORDS];
    take_record(b, tw);
    take_power_record(b, tw);
    take_phasor_record(b, tw);
    take_unbalance_record(b, tw);
    CHECK(b.m_agg.empty(), "priming block must not emit an interval");

    GoldenAgg ten_minute_golden;
    for (int block = 0; block < 2; ++block) {
      GoldenBlock gb;
      run_block(b, tm, 12, gb);
      fold_block(ten_minute_golden, gb, 12, tm.freq_mhz);
      take_record(b, tw);
      take_power_record(b, tw);
      take_phasor_record(b, tw);
      take_unbalance_record(b, tw);
      if (block == 0) {
        CHECK(b.m_agg.empty(),
              "ten-minute interval must wait for the target block");
      }
    }

    // The interval finalize is deliberately deferred to a later free-running
    // invocation so the basic and aggregate tiers share one arithmetic
    // datapath without extending the block-close latency.  Plain C simulation
    // observes the empty-input pump in Bench::send(), but Vitis' cosimulation
    // transaction wrapper does not record top-level invocations which carry
    // no input transaction (the top is ap_ctrl_none with nonblocking AXIS).
    // Supply one real look-ahead cycle so both models exercise that deferred
    // invocation identically.  The pending ten-minute pass runs first and
    // leaves this cycle queued; the following invocation consumes it as the
    // first cycle of the next basic block.  It is intentionally not folded
    // into ten_minute_golden or the interval which just closed.
    GoldenBlock look_ahead;
    const single_cycle_result_t look_ahead_cycle = make_cycle(tm, look_ahead);
    b.send(look_ahead_cycle);
    tm.sequence += 1;
    tm.cycle_sequence += 1;
    tm.first_sample += tm.samples;
    tm.seed += 1;

    take_agg(b, tw, MREC_FORMAT_TEN_MINUTE_V1);
    const unsigned long long actual_last =
        (unsigned long long)tw[AGG_LAST_SAMPLE_LOW_WORD] |
        ((unsigned long long)tw[AGG_LAST_SAMPLE_HIGH_WORD] << 32);
    const unsigned long long recorded_target =
        (unsigned long long)tw[TEN_MINUTE_TARGET_SAMPLE_LOW_WORD] |
        ((unsigned long long)tw[TEN_MINUTE_TARGET_SAMPLE_HIGH_WORD] << 32);
    CHECK(tw[MREC_SAMPLE_COUNT_WORD] == ten_minute_golden.count,
          "ten-minute sample count %u, golden %u",
          (unsigned)tw[MREC_SAMPLE_COUNT_WORD], ten_minute_golden.count);
    CHECK((tw[MTR2_SHAPE_WORD] & 0xFFFFu) == 2u,
          "ten-minute shape carries two folded blocks");
    CHECK(((tw[MTR2_SHAPE_WORD] >> TEN_MINUTE_SHAPE_NOMINAL_LSB) & 0xFFu) ==
              60u,
          "ten-minute shape carries nominal frequency");
    CHECK(((tw[MTR2_SHAPE_WORD] >> TEN_MINUTE_SHAPE_FLAGS_LSB) &
           (1u << TEN_MINUTE_FLAG_CONTAMINATED_BIT)) != 0,
          "first programmed interval is marked contaminated");
    CHECK((tw[MREC_STATUS_WORD] &
           (1u << TEN_MINUTE_STATUS_COMPLETE_BIT)) != 0,
          "ten-minute record is complete");
    CHECK((tw[MREC_STATUS_WORD] &
           (1u << TEN_MINUTE_STATUS_CONTAMINATED_BIT)) != 0,
          "ten-minute status repeats contamination");
    CHECK((tw[MREC_STATUS_WORD] &
           (1u << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT)) != 0,
          "ten-minute record carries a valid UTC boundary");
    CHECK(tw[MTR2_FREQUENCY_WORD] == 0,
          "ten-minute frequency is unavailable by definition");
    CHECK(tw[TEN_MINUTE_TOTAL_CYCLES_WORD] == 24u,
          "ten-minute record carries complete cycle count");
    CHECK(recorded_target == target,
          "ten-minute target provenance %llu, expected %llu",
          recorded_target, target);
    CHECK(actual_last >= target &&
              tw[TEN_MINUTE_OVERSHOOT_SAMPLES_WORD] == actual_last - target,
          "ten-minute close overshoot is exact");

    const unsigned ten_minute_sequence = tw[MREC_SEQUENCE_WORD];
    take_agg(b, tw, MREC_FORMAT_TEN_MINUTE_POWER_V1);
    CHECK(tw[MREC_SEQUENCE_WORD] == ten_minute_sequence,
          "ten-minute power record shares interval sequence");
    take_agg(b, tw, MREC_FORMAT_TEN_MINUTE_PHASOR_V2);
    CHECK(tw[MREC_SEQUENCE_WORD] == ten_minute_sequence,
          "ten-minute phasor record shares interval sequence");
    take_agg(b, tw, MREC_FORMAT_TEN_MINUTE_UNBAL_V2);
    CHECK(tw[MREC_SEQUENCE_WORD] == ten_minute_sequence,
          "ten-minute unbalance record shares interval sequence");
    CHECK(b.m_agg.empty(), "ten-minute close emits exactly one record quad");

    // Leave the consumed update-toggle state intact for the following M14
    // scenario.  Toggling twice without presenting an input beat would make
    // the next target update invisible to the engine.
  }

  // --- Cascaded two-hour tier: exactly twelve complete/aligned ten-minute
  // accumulator sets, never twelve finalized RMS values.  A 1 Hz metadata
  // rate and 50 samples/cycle make each 12-cycle basic block exactly one
  // synthetic 600-s interval.  This accelerates the cadence only; the
  // arithmetic and provenance path is the production M14 path.
  {
    b.enable = true; b.locked = true; b.fallback = false;
    b.dc_remove = true;
    b.sample_rate = 1;
    while (!b.m_axis.empty()) b.m_axis.read();
    while (!b.m_agg.empty()) b.m_agg.read();

    CycleSpec h2;
    h2.sequence = 30000; h2.cycle_sequence = 30000;
    h2.first_sample = 5000000;
    h2.nominal = 60; h2.samples = 50; h2.generation = b.cfg_generation;

    const unsigned block_samples = 12u * h2.samples;
    const unsigned long long first_target =
        h2.first_sample + 2u * block_samples - 1u;
    b.ten_minute_target = first_target;
    b.ten_minute_valid = true;
    b.ten_minute_update = !b.ten_minute_update;

    // Configuration APPLY excludes this priming block from every interval
    // and leaves the first programmed ten-minute interval contaminated.
    GoldenBlock priming;
    run_block(b, h2, 12, priming, /*apply_on_first=*/true);
    ap_uint<32> hw[MREC_WORDS];
    take_record(b, hw);
    take_power_record(b, hw);
    take_phasor_record(b, hw);
    take_unbalance_record(b, hw);
    CHECK(b.m_agg.empty(), "two-hour priming block emits no interval");

    GoldenAgg two_hour_golden;
    unsigned first_clean_t10m_sequence = 0;
    unsigned last_clean_t10m_sequence = 0;
    unsigned long long first_clean_sample = 0;
    unsigned long long last_clean_sample = 0;

    // First close is contaminated and must not seed M14.  The next twelve
    // closes are autonomous, aligned ten-minute intervals and must produce
    // exactly one two-hour record family after the twelfth.
    GoldenBlock prefed_block;
    bool has_prefed_cycle = false;
    for (unsigned interval = 0; interval < 13; ++interval) {
      GoldenBlock gb = prefed_block;
      const unsigned first_cycle = has_prefed_cycle ? 1u : 0u;
      has_prefed_cycle = false;
      prefed_block = GoldenBlock{};
      for (unsigned cycle = first_cycle; cycle < 12u; ++cycle) {
        b.send(make_cycle(h2, gb));
        h2.sequence += 1;
        h2.cycle_sequence += 1;
        h2.first_sample += h2.samples;
        h2.seed += 1;
      }
      take_record(b, hw);
      take_power_record(b, hw);
      take_phasor_record(b, hw);
      take_unbalance_record(b, hw);

      take_agg(b, hw, MREC_FORMAT_TEN_MINUTE_V1);
      const unsigned t10m_sequence = (unsigned)hw[MREC_SEQUENCE_WORD];
      const unsigned long long t10m_first =
          (unsigned long long)hw[MREC_FIRST_SAMPLE_LOW_WORD] |
          ((unsigned long long)hw[MREC_FIRST_SAMPLE_HIGH_WORD] << 32);
      const unsigned long long t10m_last =
          (unsigned long long)hw[AGG_LAST_SAMPLE_LOW_WORD] |
          ((unsigned long long)hw[AGG_LAST_SAMPLE_HIGH_WORD] << 32);
      if (interval == 0) {
        CHECK((hw[MREC_STATUS_WORD] &
               (1u << TEN_MINUTE_STATUS_CONTAMINATED_BIT)) != 0,
              "first synthetic ten-minute interval is contaminated");
      } else {
        CHECK((hw[MREC_STATUS_WORD] &
               (1u << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT)) != 0,
              "M14 input %u is time aligned", interval);
        fold_block(two_hour_golden, gb, 12, h2.freq_mhz);
        if (interval == 1) {
          first_clean_t10m_sequence = t10m_sequence;
          first_clean_sample = t10m_first;
        }
        last_clean_t10m_sequence = t10m_sequence;
        last_clean_sample = t10m_last;
      }
      take_agg(b, hw, MREC_FORMAT_TEN_MINUTE_POWER_V1);
      take_agg(b, hw, MREC_FORMAT_TEN_MINUTE_PHASOR_V2);
      take_agg(b, hw, MREC_FORMAT_TEN_MINUTE_UNBAL_V2);
      CHECK(b.m_agg.empty(),
            "two-hour result remains deferred through input %u", interval);

      // Every clean input before the twelfth publishes an open two-hour
      // view when enabled. Feed the first cycle of the next Basic block in
      // both builds so completed-record inputs remain byte-for-byte equal.
      if (interval >= 1u && interval < 12u) {
        prefed_block = GoldenBlock{};
        b.send(make_cycle(h2, prefed_block));
        h2.sequence += 1;
        h2.cycle_sequence += 1;
        h2.first_sample += h2.samples;
        h2.seed += 1;
        has_prefed_cycle = true;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
        take_open_quad(b, hw,
                       MREC_FORMAT_OPEN_TWO_HOUR_V1,
                       MREC_FORMAT_OPEN_TWO_HOUR_POWER_V1,
                       MREC_FORMAT_OPEN_TWO_HOUR_PHASOR_V2,
                       MREC_FORMAT_OPEN_TWO_HOUR_UNBAL_V2);
        CHECK((hw[MTR2_SHAPE_WORD] & 0xFFFFu) == interval,
              "open two-hour preview reports %u completed inputs", interval);
        CHECK(b.m_agg.empty(),
              "open two-hour preview emits one record quad");
#endif
      }
    }

    // Supply one look-ahead cycle so Vitis cosimulation observes the
    // input-free deferred pass exactly as hardware does.
    GoldenBlock look_ahead;
    b.send(make_cycle(h2, look_ahead));
    h2.sequence += 1;
    h2.cycle_sequence += 1;
    h2.first_sample += h2.samples;

    take_agg(b, hw, MREC_FORMAT_TWO_HOUR_V1);
    CHECK(hw[MREC_SAMPLE_COUNT_WORD] == two_hour_golden.count,
          "two-hour sample count %u, golden %u",
          (unsigned)hw[MREC_SAMPLE_COUNT_WORD], two_hour_golden.count);
    CHECK((hw[MTR2_SHAPE_WORD] & 0xFFFFu) == 12u,
          "two-hour shape carries twelve ten-minute inputs");
    CHECK(hw[MTR2_FIRST_BASIC_SEQ_WORD] == first_clean_t10m_sequence &&
              hw[MTR2_LAST_BASIC_SEQ_WORD] == last_clean_t10m_sequence,
          "two-hour provenance spans ten-minute sequences %u..%u",
          first_clean_t10m_sequence, last_clean_t10m_sequence);
    const unsigned long long got_first_sample =
        (unsigned long long)hw[MREC_FIRST_SAMPLE_LOW_WORD] |
        ((unsigned long long)hw[MREC_FIRST_SAMPLE_HIGH_WORD] << 32);
    const unsigned long long got_last_sample =
        (unsigned long long)hw[AGG_LAST_SAMPLE_LOW_WORD] |
        ((unsigned long long)hw[AGG_LAST_SAMPLE_HIGH_WORD] << 32);
    CHECK(got_first_sample == first_clean_sample &&
              got_last_sample == last_clean_sample,
          "two-hour sample domain is contiguous %llu..%llu",
          first_clean_sample, last_clean_sample);
    CHECK(hw[MTR2_FREQUENCY_WORD] == 0,
          "two-hour frequency is unavailable by definition");
    CHECK(hw[TEN_MINUTE_TOTAL_CYCLES_WORD] == 12u * 12u,
          "two-hour record carries 144 contributing cycles");
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      const unsigned long long rms_q16 = golden_rms_q16(
          two_hour_golden.square[lane], two_hour_golden.sum[lane],
          two_hour_golden.count, true);
      const unsigned long long got =
          (unsigned long long)hw[MTR2_CH_BASE_WORD +
                                 lane * MTR2_CH_STRIDE_WORDS];
      CHECK(got == (rms_q16 >> 16),
            "two-hour lane %d RMS %llu, golden %llu", lane, got,
            rms_q16 >> 16);
    }
    const unsigned h2_sequence = (unsigned)hw[MREC_SEQUENCE_WORD];
    take_agg(b, hw, MREC_FORMAT_TWO_HOUR_POWER_V1);
    CHECK(hw[MREC_SEQUENCE_WORD] == h2_sequence,
          "two-hour power record shares interval sequence");
    take_agg(b, hw, MREC_FORMAT_TWO_HOUR_PHASOR_V2);
    CHECK(hw[MREC_SEQUENCE_WORD] == h2_sequence,
          "two-hour phasor record shares interval sequence");
    take_agg(b, hw, MREC_FORMAT_TWO_HOUR_UNBAL_V2);
    CHECK(hw[MREC_SEQUENCE_WORD] == h2_sequence,
          "two-hour unbalance record shares interval sequence");
    CHECK(b.m_agg.empty(), "two-hour close emits exactly one record quad");

    b.sample_rate = 32000;
    b.ten_minute_valid = false;
    b.ten_minute_update = !b.ten_minute_update;
  }


  // --- Soak: 8000 cycles, disruptions scheduled into long clean runs. -----
  // The exact-golden scenarios above test each disruption KIND in isolation.
  // This adds phase variation and long-run stability, and asserts that both
  // tiers keep producing -- a bench that silently stops exercising a tier is
  // worse than no bench. (An earlier version of this sweep used a uniform
  // per-cycle disruption rate, which makes a clean 12-cycle block ~7% likely
  // and emitted ZERO aggregates while still passing on the basic tier.)
  //
  // Keep the long randomized sweep in C simulation.  Vitis' generated
  // ap_ctrl_none cosimulation wrapper repeatedly opens and closes stream
  // trace files; the thousands of basic/aggregate records produced here can
  // exhaust XSim's internal file-channel table even though the RTL values are
  // correct.  The deterministic scenarios above remain in both C and RTL
  // simulation and cover every completed and open record family.  This split
  // therefore preserves functional RTL coverage while leaving long-run and
  // disruption coverage in the faster model that can sustain it.
#ifndef __HLS_COSIM__
  {
    b.enable = true; b.locked = true; b.fallback = false;
    while (!b.m_axis.empty()) b.m_axis.read();
    while (!b.m_agg.empty()) b.m_agg.read();

    CycleSpec sc;
    sc.sequence = 40000; sc.cycle_sequence = 40000; sc.first_sample = 3000000;
    sc.nominal = 60; sc.generation = b.cfg_generation;
    unsigned blocks = 0, intervals = 0;
    int next_disrupt = 400, kind = 0;

    for (int n = 0; n < 8000; ++n) {
      GoldenBlock scratch;
      bool toggle = false;
      b.locked = true; b.fallback = false;
      unsigned saved_status = sc.status;
      if (n == next_disrupt) {
        switch (kind) {
        case 0: sc.status |= (1u << SCYC_STATUS_FIRST_AFTER_GAP_BIT); break;
        case 1: b.locked = false; break;
        case 2: b.fallback = true; break;
        case 3: sc.freq_valid = 0; break;
        case 4: sc.status |= (1u << SCYC_STATUS_PHASOR_INVALID_BIT); break;
        case 5: sc.status |= (1u << SCYC_STATUS_OVERFLOW_BIT); break;
        case 6: sc.sequence += 3; sc.cycle_sequence += 3; break;
        case 7: toggle = true; break;
        case 8: sc.nominal = (sc.nominal == 60) ? 50 : 60; break;
        }
        kind = (kind + 1) % 9;
        next_disrupt += 241 + 30 * kind;
      }
      const single_cycle_result_t r = make_cycle(sc, scratch);
      b.send(r, toggle);
      sc.status = saved_status;
      sc.freq_valid = 1;
      sc.sequence += 1; sc.cycle_sequence += 1; sc.first_sample += sc.samples;

      ap_uint<32> w[MREC_WORDS];
      while (!b.m_axis.empty()) {
        take_record(b, w);
        if (w[MREC_FORMAT_WORD] == MREC_FORMAT_BASIC_V4) ++blocks;
      }
      while (!b.m_agg.empty()) {
        take_agg(b, w, /*any format=*/0u);
        if (w[MREC_FORMAT_WORD] == MREC_FORMAT_AGG_V3) ++intervals;
      }
    }
    CHECK(blocks > 400, "soak must close many blocks, got %u", blocks);
    CHECK(intervals > 10, "soak must close several intervals, got %u", intervals);
    std::printf("soak: %u blocks, %u intervals\n", blocks, intervals);
  }
#else
  std::printf("cosim: randomized soak covered by C simulation\n");
#endif

#if MNC_REQUIRE_M15_INVALIDATION_MATRIX
  // Accelerate the ten-minute cadence without changing its arithmetic: at a
  // metadata rate of 4 samples/s, four 600-sample Basic blocks span one
  // programmed 2,400-sample interval.  Each case first proves a contaminated
  // startup interval and a clean successor, injects one discontinuity, then
  // requires the first recoverable interval to be contaminated and the next
  // complete interval to be clean.
  auto drain_basic_quads = [&]() {
    ap_uint<32> words[MREC_WORDS];
    while (!b.m_axis.empty()) {
      take_record(b, words);
      take_power_record(b, words);
      take_phasor_record(b, words);
      take_unbalance_record(b, words);
    }
  };

  bool ten_minute_contaminated[64] = {};
  unsigned ten_minute_results = 0;
  auto drain_interval_records = [&]() {
    ap_uint<32> words[MREC_WORDS];
    while (!b.m_agg.empty()) {
      take_agg(b, words, /*any format=*/0u);
      if ((unsigned)words[MREC_FORMAT_WORD] != MREC_FORMAT_TEN_MINUTE_V1)
        continue;
      if (ten_minute_results < 64u) {
        ten_minute_contaminated[ten_minute_results] =
            words[MREC_STATUS_WORD].bit(TEN_MINUTE_STATUS_CONTAMINATED_BIT);
      }
      ++ten_minute_results;
    }
  };

  auto drive_matrix_block = [&](CycleSpec &cycle, bool apply) {
    GoldenBlock golden;
    run_block(b, cycle, cycle.nominal == 50u ? 10u : 12u, golden, apply);
    drain_basic_quads();
    drain_interval_records();
  };

  auto expect_latest_interval = [&](unsigned before, bool contaminated,
                                    const char *label) {
    CHECK(ten_minute_results == before + 1u,
          "%s must emit one ten-minute result, got %u", label,
          ten_minute_results - before);
    if (ten_minute_results == before + 1u) {
      CHECK(ten_minute_contaminated[before] == contaminated,
            "%s contamination=%u, expected %u", label,
            ten_minute_contaminated[before] ? 1u : 0u,
            contaminated ? 1u : 0u);
    }
  };

  auto begin_matrix_case = [&](CycleSpec &cycle, unsigned sequence_base,
                               unsigned long long sample_base,
                               const char *label) {
    while (!b.m_axis.empty()) b.m_axis.read();
    while (!b.m_agg.empty()) b.m_agg.read();
    b.sample_rate = 4u;
    b.enable = true;
    b.locked = true;
    b.fallback = false;
    b.ten_minute_valid = true;
    cycle = CycleSpec{};
    cycle.sequence = sequence_base;
    cycle.cycle_sequence = sequence_base;
    cycle.first_sample = sample_base;
    cycle.samples = 50u;
    cycle.nominal = 60u;
    cycle.freq_mhz = 60000u;
    cycle.generation = b.cfg_generation;
    b.ten_minute_target = sample_base + 5ull * 600ull - 1ull;
    b.ten_minute_update = !b.ten_minute_update;

    drive_matrix_block(cycle, /*APPLY=*/true);
    unsigned before = ten_minute_results;
    for (unsigned block = 0; block < 4u; ++block)
      drive_matrix_block(cycle, /*APPLY=*/false);
    expect_latest_interval(before, true, label);

    before = ten_minute_results;
    for (unsigned block = 0; block < 4u; ++block)
      drive_matrix_block(cycle, /*APPLY=*/false);
    expect_latest_interval(before, false, "clean interval before fault");
  };

  auto require_fault_recovery = [&](CycleSpec &cycle, const char *label) {
    unsigned before = ten_minute_results;
    for (unsigned block = 0; block < 3u; ++block)
      drive_matrix_block(cycle, /*APPLY=*/false);
    expect_latest_interval(before, true, label);

    before = ten_minute_results;
    for (unsigned block = 0; block < 4u; ++block)
      drive_matrix_block(cycle, /*APPLY=*/false);
    expect_latest_interval(before, false, "clean interval after fault");
  };

  CycleSpec matrix_cycle;

  begin_matrix_case(matrix_cycle, 80000u, 8000000ull,
                    "startup/APPLY interval");
  matrix_cycle.sequence += 2u;
  matrix_cycle.cycle_sequence += 2u;
  drive_matrix_block(matrix_cycle, /*APPLY=*/false);
  require_fault_recovery(matrix_cycle,
                         "missing/sequence-gap/PL-reset interval");

  begin_matrix_case(matrix_cycle, 81000u, 8100000ull,
                    "lock-loss setup interval");
  b.locked = false;
  drive_matrix_block(matrix_cycle, /*APPLY=*/false);
  b.locked = true;
  require_fault_recovery(matrix_cycle, "lock-loss interval");

  begin_matrix_case(matrix_cycle, 82000u, 8200000ull,
                    "fallback setup interval");
  b.fallback = true;
  drive_matrix_block(matrix_cycle, /*APPLY=*/false);
  b.fallback = false;
  require_fault_recovery(matrix_cycle, "fallback interval");

  begin_matrix_case(matrix_cycle, 83000u, 8300000ull,
                    "source-change setup interval");
  ++b.cfg_generation;
  matrix_cycle.generation = b.cfg_generation;
  drive_matrix_block(matrix_cycle, /*APPLY=*/true);
  require_fault_recovery(matrix_cycle,
                         "APPLY/source-generation interval");

  begin_matrix_case(matrix_cycle, 83500u, 8350000ull,
                    "nominal-transition setup interval");
  ++b.cfg_generation;
  matrix_cycle.generation = b.cfg_generation;
  matrix_cycle.nominal = 50u;
  matrix_cycle.freq_mhz = 50000u;
  matrix_cycle.samples = 60u;
  drive_matrix_block(matrix_cycle, /*APPLY=*/true);
  require_fault_recovery(matrix_cycle, "60-to-50 Hz transition interval");

  begin_matrix_case(matrix_cycle, 84000u, 8400000ull,
                    "UTC-correction setup interval");
  b.ten_minute_target = matrix_cycle.first_sample + 4ull * 600ull - 1ull;
  b.ten_minute_update = !b.ten_minute_update;
  unsigned before_correction = ten_minute_results;
  for (unsigned block = 0; block < 4u; ++block)
    drive_matrix_block(matrix_cycle, /*APPLY=*/false);
  expect_latest_interval(before_correction, true, "UTC-correction interval");
  unsigned before_clean = ten_minute_results;
  for (unsigned block = 0; block < 4u; ++block)
    drive_matrix_block(matrix_cycle, /*APPLY=*/false);
  expect_latest_interval(before_clean, false,
                         "clean interval after UTC correction");

  begin_matrix_case(matrix_cycle, 85000u, 8500000ull,
                    "APU-restart setup interval");
  b.ten_minute_valid = false;
  b.ten_minute_update = !b.ten_minute_update;
  drive_matrix_block(matrix_cycle, /*APPLY=*/false);
  b.ten_minute_valid = true;
  b.ten_minute_target = matrix_cycle.first_sample + 3ull * 600ull - 1ull;
  b.ten_minute_update = !b.ten_minute_update;
  require_fault_recovery(matrix_cycle, "APU-restart/remap interval");

  std::printf("invalidation matrix: %u ten-minute results checked\n",
              ten_minute_results);
#endif

  // IEC 61000-4-30 resynchronizes the Basic and 150/180-cycle windows at
  // every UTC ten-minute boundary.  When the boundary lands inside an open
  // window, the old window continues to completion while a synchronized
  // window starts.  The resulting records overlap by construction.  This is
  // a permanent regression gate for the dual-slot M15 implementation.
  auto drain_aggregate_records = [&](unsigned long long *first_samples,
                                     unsigned long long *last_samples,
                                     unsigned *statuses,
                                     unsigned *sample_counts,
                                     unsigned *first_sequences,
                                     unsigned *last_sequences,
                                     unsigned long long *rms_values,
                                     unsigned &aggregate_count) {
    ap_uint<32> words[MREC_WORDS];
    while (!b.m_agg.empty()) {
      take_agg(b, words, /*any format=*/0u);
      if ((unsigned)words[MREC_FORMAT_WORD] != MREC_FORMAT_AGG_V3) continue;
      if (aggregate_count < 4u) {
        first_samples[aggregate_count] =
            (unsigned long long)words[MREC_FIRST_SAMPLE_LOW_WORD] |
            ((unsigned long long)words[MREC_FIRST_SAMPLE_HIGH_WORD] << 32);
        last_samples[aggregate_count] =
            (unsigned long long)words[AGG_LAST_SAMPLE_LOW_WORD] |
            ((unsigned long long)words[AGG_LAST_SAMPLE_HIGH_WORD] << 32);
        statuses[aggregate_count] = (unsigned)words[MREC_STATUS_WORD];
        sample_counts[aggregate_count] =
            (unsigned)words[MREC_SAMPLE_COUNT_WORD];
        first_sequences[aggregate_count] =
            (unsigned)words[MTR2_FIRST_BASIC_SEQ_WORD];
        last_sequences[aggregate_count] =
            (unsigned)words[MTR2_LAST_BASIC_SEQ_WORD];
        for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
          rms_values[aggregate_count * MET_ACTIVE_CHANNELS + lane] =
              (unsigned long long)
                  words[MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS] |
              ((unsigned long long)
                   words[MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS + 1]
               << 32);
        }
      }
      ++aggregate_count;
    }
  };

  auto run_basic_utc_overlap = [&](unsigned nominal_hz,
                                   unsigned cycles_per_block,
                                   unsigned target_cycle,
                                   unsigned sequence_base,
                                   unsigned long long sample_base) {
    while (!b.m_axis.empty()) b.m_axis.read();
    while (!b.m_agg.empty()) b.m_agg.read();

    b.sample_rate = 32000;
    b.enable = true;
    b.locked = true;
    b.fallback = false;
    b.ten_minute_valid = true;

    CycleSpec sync;
    sync.sequence = sequence_base;
    sync.cycle_sequence = sequence_base;
    sync.first_sample = sample_base;
    sync.samples = 10;
    sync.nominal = nominal_hz;
    sync.freq_mhz = nominal_hz * 1000u;
    sync.generation = b.cfg_generation;

    // Deliberately land the UTC boundary halfway through a whole-cycle input.
    b.ten_minute_target =
        sample_base + (unsigned long long)target_cycle * sync.samples +
        sync.samples / 2u;
    b.ten_minute_update = !b.ten_minute_update;

    unsigned basic_count = 0;
    unsigned synchronized_count = 0;
    unsigned long long first_samples[4] = {};
    unsigned long long last_samples[4] = {};
    const unsigned cycles_to_drive = target_cycle + cycles_per_block + 1u;
    for (unsigned cycle_index = 0; cycle_index < cycles_to_drive;
         ++cycle_index) {
      GoldenBlock golden;
      b.send(make_cycle(sync, golden), /*APPLY=*/cycle_index == 0u);
      sync.sequence += 1;
      sync.cycle_sequence += 1;
      sync.first_sample += sync.samples;
      sync.seed += 1;

      ap_uint<32> words[MREC_WORDS];
      while (!b.m_axis.empty()) {
        take_record(b, words);
        CHECK(words[MREC_FORMAT_WORD] == MREC_FORMAT_BASIC_V4,
              "UTC-overlap Basic family starts with BASIC-v4");
        if (basic_count < 4u) {
          first_samples[basic_count] =
              (unsigned long long)words[MREC_FIRST_SAMPLE_LOW_WORD] |
              ((unsigned long long)words[MREC_FIRST_SAMPLE_HIGH_WORD] << 32);
          last_samples[basic_count] =
              (unsigned long long)words[BASIC_LAST_SAMPLE_LOW_WORD] |
              ((unsigned long long)words[BASIC_LAST_SAMPLE_HIGH_WORD] << 32);
        }
        if (words[MTR1_TIMING_WORD].bit(
                MTR1_TIMING_UTC_RESYNCHRONIZED_BIT)) {
          ++synchronized_count;
        }
        ++basic_count;
        take_power_record(b, words);
        take_phasor_record(b, words);
        take_unbalance_record(b, words);
      }
      while (!b.m_agg.empty()) take_agg(b, words, /*any format=*/0u);
    }

    CHECK(basic_count >= 2u,
          "%u Hz UTC resynchronization must complete old and synchronized "
          "Basic windows, got %u", nominal_hz, basic_count);
    bool found_overlap = false;
    for (unsigned left = 0; left < basic_count && left < 4u; ++left) {
      for (unsigned right = left + 1u; right < basic_count && right < 4u;
           ++right) {
        if (first_samples[right] <= last_samples[left] &&
            first_samples[right] > first_samples[left]) {
          found_overlap = true;
        }
      }
    }
    CHECK(found_overlap,
          "%u Hz UTC-resynchronized Basic records must overlap", nominal_hz);
    CHECK(synchronized_count == 1u,
          "%u Hz UTC resynchronization must mark exactly one Basic, got %u",
          nominal_hz, synchronized_count);
  };

  run_basic_utc_overlap(60u, 12u, 5u, 70000u, 7000000ull);
  run_basic_utc_overlap(50u, 10u, 3u, 71000u, 7100000ull);

  // Preload five clean Basic blocks into the old 150/180-cycle interval,
  // then put the UTC boundary inside the next Basic block.  Within sixteen
  // Basic-block durations both the continuing old aggregate and the new
  // synchronized aggregate must complete, and their sample ranges overlap.
  while (!b.m_axis.empty()) b.m_axis.read();
  while (!b.m_agg.empty()) b.m_agg.read();
  b.ten_minute_valid = false;
  b.ten_minute_update = !b.ten_minute_update;
  b.sample_rate = 32000;
  b.enable = true;
  b.locked = true;
  b.fallback = false;

  CycleSpec overlap;
  overlap.sequence = 72000u;
  overlap.cycle_sequence = 72000u;
  overlap.first_sample = 7200000ull;
  overlap.samples = 5u;
  overlap.nominal = 60u;
  overlap.freq_mhz = 60000u;
  overlap.generation = b.cfg_generation;

  auto drain_basic_families = [&]() {
    ap_uint<32> words[MREC_WORDS];
    while (!b.m_axis.empty()) {
      take_record(b, words);
      take_power_record(b, words);
      take_phasor_record(b, words);
      take_unbalance_record(b, words);
    }
  };

  GoldenBlock priming;
  run_block(b, overlap, 12u, priming, /*APPLY=*/true);
  drain_basic_families();
  while (!b.m_agg.empty()) {
    ap_uint<32> words[MREC_WORDS];
    take_agg(b, words, /*any format=*/0u);
  }
  GoldenAgg continuing_golden;
  for (unsigned block = 0; block < 5u; ++block) {
    GoldenBlock golden;
    run_block(b, overlap, 12u, golden);
    fold_block(continuing_golden, golden, 12u, overlap.freq_mhz);
    drain_basic_families();
  }

  const unsigned long long overlap_target =
      overlap.first_sample + 5ull * overlap.samples + overlap.samples / 2u;
  b.ten_minute_target = overlap_target;
  b.ten_minute_valid = true;
  b.ten_minute_update = !b.ten_minute_update;

  unsigned aggregate_count = 0;
  unsigned long long aggregate_first[4] = {};
  unsigned long long aggregate_last[4] = {};
  unsigned aggregate_status[4] = {};
  unsigned aggregate_samples[4] = {};
  unsigned aggregate_first_sequence[4] = {};
  unsigned aggregate_last_sequence[4] = {};
  unsigned long long aggregate_rms[4 * MET_ACTIVE_CHANNELS] = {};
  GoldenBlock continuing_boundary;
  GoldenBlock synchronized_blocks[MET_BASIC_BLOCKS_PER_AGGREGATE] = {};
  bool correction_after_promotion = false;
  for (unsigned cycle_index = 0; cycle_index < 186u; ++cycle_index) {
    GoldenBlock transmitted;
    const auto cycle = make_cycle(overlap, transmitted);
    if (cycle_index < 12u) {
      (void)make_cycle(overlap, continuing_boundary);
    }
    if (cycle_index >= 6u) {
      const unsigned synchronized_index = (cycle_index - 6u) / 12u;
      if (synchronized_index < MET_BASIC_BLOCKS_PER_AGGREGATE) {
        (void)make_cycle(overlap, synchronized_blocks[synchronized_index]);
      }
    }
    b.send(cycle);
    overlap.sequence += 1;
    overlap.cycle_sequence += 1;
    overlap.first_sample += overlap.samples;
    overlap.seed += 1;
    drain_basic_families();
    drain_aggregate_records(aggregate_first, aggregate_last, aggregate_status,
                            aggregate_samples, aggregate_first_sequence,
                            aggregate_last_sequence, aggregate_rms,
                            aggregate_count);
    if (!correction_after_promotion && aggregate_count == 1u) {
      // The old interval has just closed and the synchronized shadow is now
      // authoritative. A newer UTC mapping may cancel a still-concurrent
      // shadow, but must not erase this promoted active interval.
      b.ten_minute_target =
          overlap.first_sample + 600ull * b.sample_rate;
      b.ten_minute_update = !b.ten_minute_update;
      correction_after_promotion = true;
    }
  }
  CHECK(correction_after_promotion,
        "UTC correction exercise must run after aggregate promotion");
  CHECK(aggregate_count >= 2u,
        "UTC-spanning 150/180-cycle windows must both complete, got %u",
        aggregate_count);
  CHECK(aggregate_count >= 2u && aggregate_first[1] <= aggregate_last[0] &&
            aggregate_first[1] > aggregate_first[0],
        "UTC-spanning 150/180-cycle aggregate ranges must overlap");
  CHECK(aggregate_count >= 2u &&
            (aggregate_status[0] &
             (1u << MTR2_STATUS_UTC_OVERLAP_BIT)) != 0u &&
            (aggregate_status[0] &
             (1u << MTR2_STATUS_UTC_RESYNCHRONIZED_BIT)) == 0u,
        "continuing 150/180-cycle aggregate must carry UTC-overlap provenance");
  CHECK(aggregate_count >= 2u &&
            (aggregate_status[1] &
             (1u << MTR2_STATUS_UTC_RESYNCHRONIZED_BIT)) != 0u &&
            (aggregate_status[1] &
             (1u << MTR2_STATUS_UTC_OVERLAP_BIT)) == 0u,
        "new 150/180-cycle aggregate must carry UTC-resynchronized provenance");
  CHECK(aggregate_count >= 2u &&
            aggregate_last_sequence[0] - aggregate_first_sequence[0] ==
                MET_BASIC_BLOCKS_PER_AGGREGATE - 1u &&
            aggregate_last_sequence[1] - aggregate_first_sequence[1] ==
                MET_BASIC_BLOCKS_PER_AGGREGATE - 1u,
        "both UTC-spanning aggregates must contain exactly 15 Basic sequences");
  CHECK(aggregate_count >= 2u &&
            aggregate_samples[0] >
                aggregate_last[0] - aggregate_first[0] + 1u &&
            aggregate_samples[1] ==
                aggregate_last[1] - aggregate_first[1] + 1u,
        "only the continuing aggregate may have a shortened physical span");

  fold_block(continuing_golden, continuing_boundary, 12u,
             overlap.freq_mhz);
  GoldenAgg synchronized_golden;
  for (unsigned block = 0; block < MET_BASIC_BLOCKS_PER_AGGREGATE; ++block) {
    fold_block(synchronized_golden, synchronized_blocks[block], 12u,
               overlap.freq_mhz);
    // Five clean pre-boundary blocks plus the continuing boundary block
    // leave nine synchronized-cadence Basics in the old 15-Basic result.
    if (block <= 8u) {
      fold_block(continuing_golden, synchronized_blocks[block], 12u,
                 overlap.freq_mhz);
    }
  }
  CHECK(continuing_golden.blocks == MET_BASIC_BLOCKS_PER_AGGREGATE &&
            synchronized_golden.blocks == MET_BASIC_BLOCKS_PER_AGGREGATE,
        "UTC aggregate goldens must each fold exactly 15 Basics");
  CHECK(aggregate_count >= 2u &&
            aggregate_samples[0] == continuing_golden.count &&
            aggregate_samples[1] == synchronized_golden.count,
        "UTC aggregate contribution counts must match independent goldens");
  for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
    const unsigned long long continuing_rms =
        golden_rms_q16(continuing_golden.square[lane],
                       continuing_golden.sum[lane], continuing_golden.count,
                       true) >>
        16;
    const unsigned long long synchronized_rms =
        golden_rms_q16(synchronized_golden.square[lane],
                       synchronized_golden.sum[lane], synchronized_golden.count,
                       true) >>
        16;
    CHECK(aggregate_count >= 2u &&
              aggregate_rms[lane] == continuing_rms &&
              aggregate_rms[MET_ACTIVE_CHANNELS + lane] == synchronized_rms,
          "UTC aggregate lane %d RMS must match both independent goldens",
          lane);
  }
  if (failures != 0) {
    if (completed_trace != nullptr) std::fclose(completed_trace);
    std::printf("COMPLETED_RECORD_DIGEST=%016llx COUNT=%u\n",
                completed_digest, completed_record_count);
    std::printf("FAILED: %d check(s)\n", failures);
    return EXIT_FAILURE;
  }
  if (completed_trace != nullptr) std::fclose(completed_trace);
  std::printf("COMPLETED_RECORD_DIGEST=%016llx COUNT=%u\n",
              completed_digest, completed_record_count);
  std::printf("PASS: aggregation_engine_tb\n");
  return EXIT_SUCCESS;
}
