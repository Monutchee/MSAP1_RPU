#include "aggregation_engine.hpp"

#include "metrology_finalize.hpp"
#include "metrology_math.hpp"
#include "metrology_stats.hpp"
#include "metrology_trig.hpp"

// Deferred interval work belongs to the long-lived engine instance. It stays
// at translation-unit scope so the R5C1 wrapper can query the scheduler
// without executing empty passes.
static ap_uint<5> aggregation_interval_pending = 0;

bool hls_aggregation_engine_has_pending_work() {
  return aggregation_interval_pending != 0;
}

// Cycle-block aggregation engine. Contract, topology, beat layout and
// tier rules: see aggregation_engine.hpp.
//
// Structure: one single-shot scheduling pass:
// each invocation consumes at most one result packet; the invocation
// closing a block runs the whole finalize + every emission inline. Input
// cadence is one packet per grid cycle (~16-20 ms), leaving the firmware
// worker time to finish deferred interval work before the next packet.
//
// The derived-quantity arithmetic lives in the shared
// metrology_finalize.hpp (one definition for this tier and the 150/180
// tier); this file owns the block rules, the merge, the block-result
// beat, and the record-word assembly.

// Shared POWER payload (record words 16+). The 10/12 and 150/180 tiers have
// IDENTICAL payload maps for this record (M11), verified byte-for-byte
// before extraction, so one instance serves both; the envelope and the
// tier-specific words 13..15 stay with each caller.
static void fill_power_payload(const met_finalize_scratch_t &fin,
                               record_image_t &img) {
// Keep one runtime-shared payload writer.  Both arguments are indexed BRAM
// ports, so this remains a narrow memory interface rather than recreating the
// retired ~5,740-bit independently-enabled finalizer bus.
#pragma HLS INLINE off
  fill_power_phases:
  for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
  #pragma HLS PIPELINE off
    const int base = POWER_PHASE_BASE_WORD + phase * POWER_PHASE_STRIDE;
    const ap_uint<64> p_bits = ap_uint<64>(met_fin_phase_p_pw(fin, phase));
    img.word[base + POWER_PHASE_P_LOW] = p_bits.range(31, 0);
    img.word[base + POWER_PHASE_P_HIGH] = p_bits.range(63, 32);
    img.word[base + POWER_PHASE_S_LOW] =
        met_fin_phase_s_pva(fin, phase).range(31, 0);
    img.word[base + POWER_PHASE_S_HIGH] =
        met_fin_phase_s_pva(fin, phase).range(63, 32);
    img.word[base + POWER_PHASE_PF] =
        ap_uint<32>(met_fin_phase_pf_e6(fin, phase));
  }
  const ap_uint<64> total_p_bits = ap_uint<64>(met_fin_total_p_pw(fin));
  img.word[POWER_TOTAL_P_LOW_WORD] = total_p_bits.range(31, 0);
  img.word[POWER_TOTAL_P_HIGH_WORD] = total_p_bits.range(63, 32);
  const ap_uint<64> total_s_pva = met_fin_total_s_pva(fin);
  img.word[POWER_TOTAL_S_LOW_WORD] = total_s_pva.range(31, 0);
  img.word[POWER_TOTAL_S_HIGH_WORD] = total_s_pva.range(63, 32);
  img.word[POWER_TOTAL_PF_WORD] =
      ap_uint<32>(met_fin_total_pf_e6(fin));
  fill_power_crest:
  for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
  #pragma HLS PIPELINE off
    img.word[POWER_CREST_BASE_WORD + lane] = met_fin_crest_e4(fin, lane);
  }
}

// Shared PHASOR payload (record words 16+). The 10/12 and 150/180 tiers have
// IDENTICAL payload maps for this record (M11), verified byte-for-byte
// before extraction, so one instance serves both; the envelope and the
// tier-specific words 13..15 stay with each caller.
static void fill_phasor_payload(const met_finalize_scratch_t &fin,
                                record_image_t &img) {
#pragma HLS INLINE off
  fill_phasor_lanes:
  for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
  #pragma HLS PIPELINE off
    const int base = PHASOR_CH_BASE_WORD + lane * PHASOR_CH_STRIDE;
    img.word[base + PHASOR_CH_FUND_RMS] =
        ap_uint<64>(met_fin_fund_rms_q16(fin, lane) >> 16).range(31, 0);
    const ap_int<32> rel_turns =
        met_fin_angle_turns(fin, lane) -
        met_fin_angle_turns(fin, MET_LANE_VA);
    img.word[base + PHASOR_CH_ANGLE] =
        ap_uint<32>(met_turns_to_millidegrees(rel_turns));
  }
  fill_phasor_pairs:
  for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
  #pragma HLS PIPELINE off
    const int base = PHASOR_VLL_BASE_WORD + pair * PHASOR_VLL_STRIDE;
    img.word[base + PHASOR_CH_FUND_RMS] =
        ap_uint<64>(met_fin_vll_fund_rms_q16(fin, pair) >> 16).range(31, 0);
    const ap_int<32> rel_turns =
        met_fin_vll_angle_turns(fin, pair) -
        met_fin_angle_turns(fin, MET_LANE_VA);
    img.word[base + PHASOR_CH_ANGLE] =
        ap_uint<32>(met_turns_to_millidegrees(rel_turns));
  }
  fill_phasor_phases:
  for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
  #pragma HLS PIPELINE off
    img.word[PHASOR_DISP_BASE_WORD + phase] =
        ap_uint<32>(met_turns_to_millidegrees(
            met_fin_disp_turns(fin, phase)));
    const ap_uint<64> q1_bits =
        ap_uint<64>(met_fin_phase_q1_pvar(fin, phase));
    img.word[PHASOR_Q1_BASE_WORD + phase * 2] = q1_bits.range(31, 0);
    img.word[PHASOR_Q1_BASE_WORD + phase * 2 + 1] =
        q1_bits.range(63, 32);
    img.word[PHASOR_DPF_BASE_WORD + phase] =
        ap_uint<32>(met_fin_phase_dpf_e6(fin, phase));
    const ap_uint<64> p1_bits =
        ap_uint<64>(met_fin_phase_p1_pw(fin, phase));
    img.word[PHASOR_P1_BASE_WORD + phase * 2] = p1_bits.range(31, 0);
    img.word[PHASOR_P1_BASE_WORD + phase * 2 + 1] =
        p1_bits.range(63, 32);
  }
  const ap_uint<64> q1_total_bits = ap_uint<64>(met_fin_total_q1_pvar(fin));
  img.word[PHASOR_Q1_TOTAL_LOW_WORD] = q1_total_bits.range(31, 0);
  img.word[PHASOR_Q1_TOTAL_HIGH_WORD] = q1_total_bits.range(63, 32);
  img.word[PHASOR_DPF_TOTAL_WORD] =
      ap_uint<32>(met_fin_total_dpf_e6(fin));
  img.word[PHASOR_FLAGS_WORD] =
      (ap_uint<32>(met_fin_phase_nature(fin, 0))
       << PHASOR_FLAGS_NATURE_A_LSB) |
      (ap_uint<32>(met_fin_phase_nature(fin, 1))
       << PHASOR_FLAGS_NATURE_B_LSB) |
      (ap_uint<32>(met_fin_phase_nature(fin, 2))
       << PHASOR_FLAGS_NATURE_C_LSB) |
      (ap_uint<32>(met_fin_total_nature(fin))
       << PHASOR_FLAGS_NATURE_TOTAL_LSB) |
      (ap_uint<32>(met_fin_angle_ref_valid(fin))
       << PHASOR_FLAGS_REF_VALID_BIT);
  const ap_uint<64> p1_total_bits = ap_uint<64>(met_fin_total_p1_pw(fin));
  img.word[PHASOR_P1_TOTAL_LOW_WORD] = p1_total_bits.range(31, 0);
  img.word[PHASOR_P1_TOTAL_HIGH_WORD] = p1_total_bits.range(63, 32);
}

// Shared UNBAL payload (record words 16+). The 10/12 and 150/180 tiers have
// IDENTICAL payload maps for this record (M11), verified byte-for-byte
// before extraction, so one instance serves both; the envelope and the
// tier-specific words 13..15 stay with each caller.
static void fill_unbal_payload(const met_finalize_scratch_t &fin,
                               record_image_t &img) {
#pragma HLS INLINE off
  fill_unbal_sets:
  for (int set = 0; set < 2; ++set) {
  #pragma HLS PIPELINE off
  fill_unbal_terms:
    for (int component = 0; component < 3; ++component) {
  #pragma HLS PIPELINE off
      const int base = ((set == 0) ? UNBAL_V_BASE_WORD : UNBAL_I_BASE_WORD) +
                       component * UNBAL_SEQ_STRIDE;
      img.word[base + UNBAL_SEQ_RMS] =
          ap_uint<64>(met_fin_seq_rms_q16(fin, set, component) >> 16)
              .range(31, 0);
      const ap_int<32> rel_turns =
          met_fin_seq_angle_turns(fin, set, component) -
          met_fin_angle_turns(fin, MET_LANE_VA);
      img.word[base + UNBAL_SEQ_ANGLE] =
          ap_uint<32>(met_turns_to_millidegrees(rel_turns));
    }
    const int zero_word =
        (set == 0) ? UNBAL_V_ZERO_RATIO_WORD : UNBAL_I_ZERO_RATIO_WORD;
    const int unbal_word =
        (set == 0) ? UNBAL_V_UNBALANCE_WORD : UNBAL_I_UNBALANCE_WORD;
    img.word[zero_word] = met_fin_seq_zero_ratio_e6(fin, set);
    img.word[unbal_word] = met_fin_seq_unbal_ratio_e6(fin, set);
  }
  img.word[UNBAL_FLAGS_WORD] =
      (ap_uint<32>(met_fin_seq_set_valid(fin, 0))
       << UNBAL_FLAGS_V_VALID_BIT) |
      (ap_uint<32>(met_fin_seq_set_valid(fin, 1))
       << UNBAL_FLAGS_I_VALID_BIT) |
      (ap_uint<32>(met_fin_angle_ref_valid(fin))
       << UNBAL_FLAGS_REF_VALID_BIT);
}

#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
// Emit a non-normative view of an accumulator that is still open.  The
// arithmetic is supplied by the shared finalizer, so the preview is an exact
// view of the same sufficient statistics that will eventually produce the
// immutable completed record.  The four record-format values are runtime
// inputs on purpose: this leaves one formatter/serializer implementation for
// both open tiers instead of two separately specialized template datapaths.
static void emit_open_interval_records(
    const met_finalize_scratch_t &fin, record_image_t &image,
    ap_uint<32> fundamental_format,
    ap_uint<32> power_format, ap_uint<32> phasor_format,
    ap_uint<32> unbalance_format, ap_uint<32> sequence,
    ap_uint<32> generation, ap_uint<32> sample_rate,
    ap_uint<32> sample_count, ap_uint<8> valid_mask,
    ap_uint<32> shape_word, ap_uint<64> first_sample,
    ap_uint<64> last_sample, ap_uint<32> first_sequence,
    ap_uint<32> last_sequence, ap_uint<32> total_cycles,
    ap_uint<64> expected_end, ap_uint<32> reset_count,
    ap_uint<32> ineligible_count, ap_uint<32> continuity_count,
    ap_uint<1> arithmetic_flag, ap_uint<1> phasor_invalid,
    ap_uint<1> time_aligned, ap_uint<1> contaminated,
    ap_uint<1> boundary_valid, hls::stream<record_axis_t> &output) {
// Ten-minute and two-hour previews have the same layout and differ only in
// runtime metadata/format values.  One low-cadence formatter serves both.
#pragma HLS INLINE off
  const ap_uint<32> status =
      (ap_uint<32>(arithmetic_flag) << MREC_STATUS_ARITHMETIC_BIT) |
      (ap_uint<32>(time_aligned) << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) |
      (ap_uint<32>(contaminated) << TEN_MINUTE_STATUS_CONTAMINATED_BIT) |
      (ap_uint<32>(boundary_valid) << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT) |
      (ap_uint<32>(1) << TEN_MINUTE_STATUS_OPEN_INTERVAL_BIT) |
      (ap_uint<32>(1) << TEN_MINUTE_STATUS_NON_NORMATIVE_BIT);
  const ap_uint<32> phasor_status =
      status |
      (ap_uint<32>(phasor_invalid) << PHASOR_STATUS_INVALID_BIT);

  // All four sibling records are emitted sequentially through the caller's
  // one shared record image.
  clear_record(image);
  fill_envelope(image, sequence, generation, sample_rate, sample_count,
                valid_mask, status, first_sample);
  image.word[MTR2_SHAPE_WORD] = shape_word;
  image.word[MTR2_FIRST_BASIC_SEQ_WORD] = first_sequence;
  image.word[MTR2_LAST_BASIC_SEQ_WORD] = last_sequence;
open_record_lanes:
  for (int lane = 0; lane < MET_CHANNEL_LANES; ++lane) {
#pragma HLS PIPELINE off
    if (lane < MET_ACTIVE_CHANNELS) {
      const ap_uint<64> rms_units = met_fin_rms_q16(fin, lane) >> 16;
      const int base = MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS;
      image.word[base + 0] = rms_units.range(31, 0);
      image.word[base + 1] = rms_units.range(63, 32);
    }
  }
  image.word[MTR2_RESET_COUNT_WORD] = reset_count;
  image.word[MTR2_INELIGIBLE_COUNT_WORD] = ineligible_count;
  image.word[MTR2_CONTINUITY_COUNT_WORD] = continuity_count;
  image.word[AGG_LAST_SAMPLE_LOW_WORD] = last_sample.range(31, 0);
  image.word[AGG_LAST_SAMPLE_HIGH_WORD] = last_sample.range(63, 32);
open_record_pairs:
  for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
#pragma HLS PIPELINE off
    image.word[AGG_VLL_BASE_WORD + pair] =
        ap_uint<64>(met_fin_vll_rms(fin, pair) >> 16).range(31, 0);
  }
  image.word[TEN_MINUTE_TOTAL_CYCLES_WORD] = total_cycles;
  image.word[TEN_MINUTE_TARGET_SAMPLE_LOW_WORD] =
      expected_end.range(31, 0);
  image.word[TEN_MINUTE_TARGET_SAMPLE_HIGH_WORD] =
      expected_end.range(63, 32);
  // An open interval has not crossed its target, so overshoot is undefined
  // and encoded as zero. expected_end plus last_sample expresses progress.
  image.word[TEN_MINUTE_OVERSHOOT_SAMPLES_WORD] = 0;
  serialize_record_format(image, fundamental_format, output);

  clear_record(image);
  fill_envelope(image, sequence, generation, sample_rate, sample_count,
                valid_mask, status, first_sample);
  image.word[MTR2_SHAPE_WORD] = shape_word;
  image.word[MTR2_FIRST_BASIC_SEQ_WORD] = first_sequence;
  image.word[MTR2_LAST_BASIC_SEQ_WORD] = last_sequence;
  fill_power_payload(fin, image);
  serialize_record_format(image, power_format, output);

  clear_record(image);
  fill_envelope(image, sequence, generation, sample_rate, sample_count,
                valid_mask, phasor_status, first_sample);
  image.word[MTR2_SHAPE_WORD] = shape_word;
  image.word[MTR2_FIRST_BASIC_SEQ_WORD] = first_sequence;
  image.word[MTR2_LAST_BASIC_SEQ_WORD] = last_sequence;
  fill_phasor_payload(fin, image);
  serialize_record_format(image, phasor_format, output);

  clear_record(image);
  fill_envelope(image, sequence, generation, sample_rate, sample_count,
                valid_mask, phasor_status, first_sample);
  image.word[MTR2_SHAPE_WORD] = shape_word;
  image.word[MTR2_FIRST_BASIC_SEQ_WORD] = first_sequence;
  image.word[MTR2_LAST_BASIC_SEQ_WORD] = last_sequence;
  fill_unbal_payload(fin, image);
  serialize_record_format(image, unbalance_format, output);
}
#endif

void hls_aggregation_engine(hls::stream<single_cycle_word_t> &s_result,
                            hls::stream<record_axis_t> &m_basic,
                            hls::stream<record_axis_t> &m_agg) {
  // The hls::stream interface remains as a fixed-width software adapter for
  // the proven aggregation implementation. The block result consumed by the
  // 150/180-cycle tier is an internal variable.
#pragma HLS INTERFACE mode=axis port=s_result register_mode=off
#pragma HLS INTERFACE mode=axis port=m_basic
#pragma HLS INTERFACE mode=axis port=m_agg
#pragma HLS INTERFACE mode=ap_ctrl_none port=return

  // Committed configuration and block state; syn.rtl.reset=state re-zeroes
  // these on aresetn exactly like the sibling engines.
  static ap_uint<1> apply_seen = 0;
  // Set when a block close also completes an interval; the interval
  // finalize then runs on the following invocation.
  // Bits 0..2 defer completed 150/180-cycle, ten-minute, and two-hour
  // finalizes. Bits 3..4 defer non-normative open ten-minute and two-hour
  // previews. All five use the same textual finalizer call below.
  static ap_uint<32> active_generation = 0;
  static ap_uint<32> active_sample_rate = 32000;
  static ap_uint<8> active_valid_mask = 0;
  static ap_uint<1> active_enable = 0;
  static ap_uint<1> active_dc_remove = 1;
  static ap_uint<1> arithmetic_overflow = 0;  // sticky until APPLY
  static ap_uint<32> sequence = 0;            // first emitted result carries 1

  // Block assembly state.  Slot 0/1 are the normal cadence and the one
  // transient UTC-resynchronization shadow.  The active-slot selector is
  // swapped when the old block closes, so no wide accumulator image is
  // copied at the ten-minute boundary.
  static ap_uint<1> basic_active_slot = 0;
  static ap_uint<1> basic_shadow_active = 0;
  static ap_uint<1> basic_shadow_slot = 1;
  static ap_uint<8> basic_cycles_in_block[2] = {};
  static ap_uint<8> basic_cycles_target[2] = {12, 12};
  static ap_uint<8> basic_nominal[2] = {60, 60};
  static ap_uint<32> basic_sample_count[2] = {};
  static ap_uint<64> basic_first_sample[2] = {};
  static ap_uint<8> basic_mask[2] = {0x7F, 0x7F};
  static ap_uint<1> basic_locked_and[2] = {1, 1};
  static ap_uint<1> basic_fallback_or[2] = {};
  static ap_uint<1> basic_phasor_invalid[2] = {};
  static ap_uint<1> basic_utc_resynchronized[2] = {};
  static ap_uint<64> basic_utc_target_sample = 0;
  static ap_uint<1> basic_utc_target_valid = 0;
  static ap_uint<32> expected_result_seq = 0;
  static ap_uint<32> expected_cycle_seq = 0;
  static ap_uint<1> have_expectation = 0;
  // First finalized block after reset/APPLY/any discard carries the mark.
  static ap_uint<1> disc_pending = 1;
  // Any merged cycle without a usable frequency reference poisons the
  // block's phasor products (PHASOR/UNBAL record status bit 1).
  // `basic_slot` remains the slot consumed by this invocation even if a
  // close promotes the shadow for the next invocation.
  ap_uint<1> basic_slot = basic_active_slot;

  // All four aggregation tiers use one indexed bank per statistic.  The
  // first dimension selects Basic, 150/180-cycle, 10-minute, or 2-hour;
  // the second selects the channel/pair.  This preserves independent state
  // while replacing forty tiny forced-LUTRAM memories (and their wide tier
  // muxes) with ten serially accessed BRAM-backed banks.
  static const int MET_TIER_BASIC_0 = 0;
  static const int MET_TIER_BASIC_1 = 1;
  static const int MET_TIER_AGGREGATE_0 = 2;
  static const int MET_TIER_AGGREGATE_1 = 3;
  static const int MET_TIER_TEN_MINUTE = 4;
  static const int MET_TIER_TWO_HOUR = 5;
  static const int MET_TIER_COUNT = 6;

  static ap_int<128>
      met_acc_sum[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_sum type=ram_s2p impl=bram
  static ap_uint<128>
      met_acc_square[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_square type=ram_s2p impl=bram
  static ap_int<64>
      met_acc_raw_sum[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_raw_sum type=ram_s2p impl=bram
  static ap_uint<96>
      met_acc_raw_square[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_raw_square type=ram_s2p impl=bram
  static ap_uint<128>
      met_acc_vll_square[MET_TIER_COUNT][MET_VLL_PAIRS];
#pragma HLS BIND_STORAGE variable=met_acc_vll_square type=ram_s2p impl=bram
  static ap_int<128>
      met_acc_power[MET_TIER_COUNT][MET_POWER_PHASES];
#pragma HLS BIND_STORAGE variable=met_acc_power type=ram_s2p impl=bram
  static ap_int<64>
      met_acc_minimum[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_minimum type=ram_s2p impl=bram
  static ap_int<64>
      met_acc_maximum[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_maximum type=ram_s2p impl=bram
  static ap_int<128>
      met_acc_phasor_re[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_phasor_re type=ram_s2p impl=bram
  static ap_int<128>
      met_acc_phasor_im[MET_TIER_COUNT][MET_ACTIVE_CHANNELS];
#pragma HLS BIND_STORAGE variable=met_acc_phasor_im type=ram_s2p impl=bram

  // Open-aggregate bookkeeping; syn.rtl.reset=state re-zeroes on aresetn.
  static ap_uint<1> a3s_apply_seen = 0;
  static ap_uint<1> a3s_active_slot = 0;
  static ap_uint<1> a3s_shadow_active = 0;
  static ap_uint<1> a3s_shadow_slot = 1;
  static ap_uint<1> a3s_finalize_slot = 0;
  static ap_uint<5> a3s_blocks_accumulated_slot[2] = {};
  static ap_uint<32> a3s_agg_generation_slot[2] = {};
  static ap_uint<8> a3s_agg_nominal_slot[2] = {};
  static ap_uint<32> a3s_agg_sample_rate_slot[2] = {};
  static ap_uint<1> a3s_agg_dc_remove_slot[2] = {1, 1};
  static ap_uint<64> a3s_agg_first_sample_slot[2] = {};
  static ap_uint<64> a3s_agg_last_sample_slot[2] = {};
  static ap_uint<32> a3s_agg_first_seq_slot[2] = {};
  static ap_uint<32> a3s_agg_last_seq_slot[2] = {};
  static ap_uint<32> a3s_agg_total_samples_slot[2] = {};
  static ap_uint<16> a3s_agg_total_cycles_slot[2] = {};
  static ap_uint<8> a3s_mask_and_slot[2] = {};
  static ap_uint<36> a3s_freq_sum_slot[2] = {};
  static ap_uint<1> a3s_freq_all_valid_slot[2] = {};
  static ap_uint<1> a3s_arithmetic_flag_slot[2] = {};
  static ap_uint<1> a3s_phasor_invalid_or_slot[2] = {};
  static ap_uint<1> a3s_utc_overlap_slot[2] = {};
  static ap_uint<1> a3s_utc_resynchronized_slot[2] = {};
  static ap_uint<1> a3s_sync_seed_pending = 0;
  // Unsigned arithmetic wraps at 2**32 / 2**64, so sequence and sample
  // continuity survive wraparound without special cases (Mtr2 rule).
  static ap_uint<32> a3s_expected_next_seq_slot[2] = {};
  static ap_uint<64> a3s_expected_next_first_slot[2] = {};
  static ap_uint<32> a3s_out_sequence = 0;

  // Diagnostics (record-carried, words 33..35 — the AGG_* register tap).
  static ap_uint<32> a3s_reset_count = 0;
  static ap_uint<32> a3s_ineligible_count = 0;
  static ap_uint<32> a3s_continuity_count = 0;

  // M13 clock-aligned ten-minute tier.  It folds the same complete basic
  // blocks as a3s, but closes on the externally mapped UTC sample boundary.
  // The first interval after a target update is normally partial and is
  // therefore marked contaminated; subsequent targets advance by exactly
  // sample_rate * 600 samples inside PL.
  static ap_uint<1> t10m_update_seen = 0;
  static ap_uint<64> t10m_target_sample = 0;
  static ap_uint<1> t10m_target_valid = 0;
  static ap_uint<16> t10m_blocks_accumulated = 0;
  static ap_uint<32> t10m_generation = 0;
  static ap_uint<8> t10m_nominal = 0;
  static ap_uint<32> t10m_sample_rate = 0;
  static ap_uint<1> t10m_dc_remove = 1;
  static ap_uint<64> t10m_first_sample = 0;
  static ap_uint<64> t10m_last_sample = 0;
  static ap_uint<32> t10m_first_seq = 0;
  static ap_uint<32> t10m_last_seq = 0;
  static ap_uint<32> t10m_total_samples = 0;
  static ap_uint<32> t10m_total_cycles = 0;
  static ap_uint<8> t10m_mask_and = 0;
  static ap_uint<1> t10m_arithmetic_flag = 0;
  static ap_uint<1> t10m_phasor_invalid_or = 0;
  static ap_uint<1> t10m_contaminated = 1;
  static ap_uint<32> t10m_expected_next_seq = 0;
  static ap_uint<64> t10m_expected_next_first = 0;
  static ap_uint<32> t10m_out_sequence = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
  static ap_uint<32> t10m_open_sequence = 0;
#endif
  static ap_uint<32> t10m_reset_count = 0;
  static ap_uint<32> t10m_ineligible_count = 0;
  static ap_uint<32> t10m_continuity_count = 0;
  static ap_uint<64> t10m_closed_target = 0;
  static ap_uint<32> t10m_overshoot_samples = 0;

  // M14 two-hour tier. Unlike the 3 s and 10 min branches, this is the one
  // standards-defined cascaded tier: it folds exactly twelve COMPLETE,
  // ALIGNED ten-minute accumulator sets. It never re-aggregates finalized
  // RMS values and it never accepts a partial/contaminated ten-minute result.
  static const ap_uint<4> A2H_INTERVALS_TARGET = 12;
  static ap_uint<4> a2h_intervals_accumulated = 0;
  static ap_uint<32> a2h_generation = 0;
  static ap_uint<8> a2h_nominal = 0;
  static ap_uint<32> a2h_sample_rate = 0;
  static ap_uint<1> a2h_dc_remove = 1;
  static ap_uint<64> a2h_first_sample = 0;
  static ap_uint<64> a2h_last_sample = 0;
  static ap_uint<32> a2h_first_seq = 0;
  static ap_uint<32> a2h_last_seq = 0;
  static ap_uint<32> a2h_total_samples = 0;
  static ap_uint<32> a2h_total_cycles = 0;
  static ap_uint<8> a2h_mask_and = 0;
  static ap_uint<1> a2h_arithmetic_flag = 0;
  static ap_uint<1> a2h_phasor_invalid_or = 0;
  static ap_uint<32> a2h_expected_next_seq = 0;
  static ap_uint<64> a2h_expected_next_first = 0;
  static ap_uint<32> a2h_out_sequence = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
  static ap_uint<32> a2h_open_sequence = 0;
#endif
  static ap_uint<32> a2h_reset_count = 0;
  static ap_uint<32> a2h_ineligible_count = 0;
  static ap_uint<32> a2h_continuity_count = 0;
  static ap_uint<64> a2h_last_target = 0;
  static ap_uint<32> a2h_last_overshoot = 0;

  ap_uint<1> a3s_slot =
      (aggregation_interval_pending.bit(0) == 1)
          ? a3s_finalize_slot
          : a3s_active_slot;

  // Readable constant-tier aliases keep the measurement algorithm below
  // close to the standards terminology while all storage remains physically
  // indexed.  They are undefined at the end of this translation unit.
#define cycles_in_block basic_cycles_in_block[basic_slot]
#define block_cycles_target basic_cycles_target[basic_slot]
#define block_nominal basic_nominal[basic_slot]
#define block_sample_count basic_sample_count[basic_slot]
#define block_first_sample basic_first_sample[basic_slot]
#define block_mask basic_mask[basic_slot]
#define block_locked_and basic_locked_and[basic_slot]
#define block_fallback_or basic_fallback_or[basic_slot]
#define block_phasor_invalid basic_phasor_invalid[basic_slot]
#define block_utc_resynchronized basic_utc_resynchronized[basic_slot]

#define a3s_blocks_accumulated a3s_blocks_accumulated_slot[a3s_slot]
#define a3s_agg_generation a3s_agg_generation_slot[a3s_slot]
#define a3s_agg_nominal a3s_agg_nominal_slot[a3s_slot]
#define a3s_agg_sample_rate a3s_agg_sample_rate_slot[a3s_slot]
#define a3s_agg_dc_remove a3s_agg_dc_remove_slot[a3s_slot]
#define a3s_agg_first_sample a3s_agg_first_sample_slot[a3s_slot]
#define a3s_agg_last_sample a3s_agg_last_sample_slot[a3s_slot]
#define a3s_agg_first_seq a3s_agg_first_seq_slot[a3s_slot]
#define a3s_agg_last_seq a3s_agg_last_seq_slot[a3s_slot]
#define a3s_agg_total_samples a3s_agg_total_samples_slot[a3s_slot]
#define a3s_agg_total_cycles a3s_agg_total_cycles_slot[a3s_slot]
#define a3s_mask_and a3s_mask_and_slot[a3s_slot]
#define a3s_freq_sum a3s_freq_sum_slot[a3s_slot]
#define a3s_freq_all_valid a3s_freq_all_valid_slot[a3s_slot]
#define a3s_arithmetic_flag a3s_arithmetic_flag_slot[a3s_slot]
#define a3s_phasor_invalid_or a3s_phasor_invalid_or_slot[a3s_slot]
#define a3s_expected_next_seq a3s_expected_next_seq_slot[a3s_slot]
#define a3s_expected_next_first a3s_expected_next_first_slot[a3s_slot]
#define a3s_utc_overlap a3s_utc_overlap_slot[a3s_slot]
#define a3s_utc_resynchronized a3s_utc_resynchronized_slot[a3s_slot]

#define BASIC_TIER(slot) ((slot) == 0 ? MET_TIER_BASIC_0 : MET_TIER_BASIC_1)
#define acc_sum met_acc_sum[BASIC_TIER(basic_slot)]
#define acc_square met_acc_square[BASIC_TIER(basic_slot)]
#define acc_raw_sum met_acc_raw_sum[BASIC_TIER(basic_slot)]
#define acc_raw_square met_acc_raw_square[BASIC_TIER(basic_slot)]
#define acc_vll_square met_acc_vll_square[BASIC_TIER(basic_slot)]
#define acc_power met_acc_power[BASIC_TIER(basic_slot)]
#define acc_minimum met_acc_minimum[BASIC_TIER(basic_slot)]
#define acc_maximum met_acc_maximum[BASIC_TIER(basic_slot)]
#define acc_phasor_re met_acc_phasor_re[BASIC_TIER(basic_slot)]
#define acc_phasor_im met_acc_phasor_im[BASIC_TIER(basic_slot)]

#define A3S_TIER(slot) ((slot) == 0 ? MET_TIER_AGGREGATE_0 : MET_TIER_AGGREGATE_1)
#define a3s_acc_sum met_acc_sum[A3S_TIER(a3s_slot)]
#define a3s_acc_square met_acc_square[A3S_TIER(a3s_slot)]
#define a3s_acc_raw_sum met_acc_raw_sum[A3S_TIER(a3s_slot)]
#define a3s_acc_raw_square met_acc_raw_square[A3S_TIER(a3s_slot)]
#define a3s_acc_vll_square met_acc_vll_square[A3S_TIER(a3s_slot)]
#define a3s_acc_power met_acc_power[A3S_TIER(a3s_slot)]
#define a3s_acc_minimum met_acc_minimum[A3S_TIER(a3s_slot)]
#define a3s_acc_maximum met_acc_maximum[A3S_TIER(a3s_slot)]
#define a3s_acc_phasor_re met_acc_phasor_re[A3S_TIER(a3s_slot)]
#define a3s_acc_phasor_im met_acc_phasor_im[A3S_TIER(a3s_slot)]

#define t10m_acc_sum met_acc_sum[MET_TIER_TEN_MINUTE]
#define t10m_acc_square met_acc_square[MET_TIER_TEN_MINUTE]
#define t10m_acc_raw_sum met_acc_raw_sum[MET_TIER_TEN_MINUTE]
#define t10m_acc_raw_square met_acc_raw_square[MET_TIER_TEN_MINUTE]
#define t10m_acc_vll_square met_acc_vll_square[MET_TIER_TEN_MINUTE]
#define t10m_acc_power met_acc_power[MET_TIER_TEN_MINUTE]
#define t10m_acc_minimum met_acc_minimum[MET_TIER_TEN_MINUTE]
#define t10m_acc_maximum met_acc_maximum[MET_TIER_TEN_MINUTE]
#define t10m_acc_phasor_re met_acc_phasor_re[MET_TIER_TEN_MINUTE]
#define t10m_acc_phasor_im met_acc_phasor_im[MET_TIER_TEN_MINUTE]

#define a2h_acc_sum met_acc_sum[MET_TIER_TWO_HOUR]
#define a2h_acc_square met_acc_square[MET_TIER_TWO_HOUR]
#define a2h_acc_raw_sum met_acc_raw_sum[MET_TIER_TWO_HOUR]
#define a2h_acc_raw_square met_acc_raw_square[MET_TIER_TWO_HOUR]
#define a2h_acc_vll_square met_acc_vll_square[MET_TIER_TWO_HOUR]
#define a2h_acc_power met_acc_power[MET_TIER_TWO_HOUR]
#define a2h_acc_minimum met_acc_minimum[MET_TIER_TWO_HOUR]
#define a2h_acc_maximum met_acc_maximum[MET_TIER_TWO_HOUR]
#define a2h_acc_phasor_re met_acc_phasor_re[MET_TIER_TWO_HOUR]
#define a2h_acc_phasor_im met_acc_phasor_im[MET_TIER_TWO_HOUR]

  // ---- One tier per invocation (A1 latency fix) -------------------------
  // A block close and an interval close used to happen in the SAME
  // invocation: two finalizes plus eight records, 22,935 clocks worst
  // case against the 6,684 of the engine this replaced. That is 74x
  // inside the 1.67 M clocks between result packets at 60 Hz, so it was
  // never a product risk -- but the whole-chain stream bench feeds
  // samples far faster than real time, the single-cycle shim's 8-deep
  // FIFO overflowed, and the dropped beat surfaced as a spurious
  // first-after-gap mark on the third block. Deferring the interval to
  // the NEXT invocation restores the pipelining the two engines had for
  // free, and keeps the finalize at ONE call site.
  single_cycle_result_t cycle;
  // Only the shim-appended CONTEXT pass 0 needs for record words 56..63 is
  // hoisted. The result now arrives as 32-bit words, so no 7,488-bit AXIS
  // register or whole-packet selection network exists at this boundary.
  ap_uint<32> ctx_freq_status = 0, ctx_freq_period = 0, ctx_freq_seq = 0;
  ap_uint<32> ctx_cap_frames = 0, ctx_cap_hdrerr = 0, ctx_cap_overflow = 0,
              ctx_cap_alerts = 0;
  ap_uint<8> result_mask = 0;
  ap_uint<32> count_now = 0;
  ap_uint<6> pass_armed = 0;  // completed tiers plus optional open previews

  if (aggregation_interval_pending.bit(0) == 1) {
    // Deferred interval pass: consume NO beat this invocation. The
    // interval accumulators are static and nothing touches them until the
    // next block closes, so they are stable across the gap.
    aggregation_interval_pending.bit(0) = 0;
    pass_armed = 2;
  } else if (aggregation_interval_pending.bit(1) == 1) {
    aggregation_interval_pending.bit(1) = 0;
    pass_armed = 4;
  } else if (aggregation_interval_pending.bit(2) == 1) {
    aggregation_interval_pending.bit(2) = 0;
    pass_armed = 8;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
  } else if (aggregation_interval_pending.bit(3) == 1) {
    aggregation_interval_pending.bit(3) = 0;
    pass_armed = 16;
  } else if (aggregation_interval_pending.bit(4) == 1) {
    aggregation_interval_pending.bit(4) = 0;
    pass_armed = 32;
#endif
  } else {
    if (s_result.empty()) {
      return;
    }
    // The packet is fixed length. Once its first word is available, the
    // blocking reads consume exactly one complete result and its captured
    // context; an incomplete packet therefore cannot be mistaken for the
    // next grid cycle.
    cycle = read_single_cycle_packet(s_result);
    const single_cycle_word_t ctx_cfg_generation = s_result.read();
    const single_cycle_word_t ctx_cfg_rate = s_result.read();
    const single_cycle_word_t ctx_controls = s_result.read();
    ctx_freq_status = s_result.read();
    ctx_freq_period = s_result.read();
    ctx_freq_seq = s_result.read();
    ctx_cap_frames = s_result.read();
    ctx_cap_hdrerr = s_result.read();
    ctx_cap_overflow = s_result.read();
    ctx_cap_alerts = s_result.read();
    ap_uint<64> ctx_t10m_target = 0;
    ctx_t10m_target.range(31, 0) = s_result.read();
    ctx_t10m_target.range(63, 32) = s_result.read();
    const single_cycle_word_t ctx_target_controls = s_result.read();

    const ap_uint<1> beat_t10m_update =
        ctx_target_controls.bit(AGG_CONTEXT_TARGET_UPDATE_BIT);
    if (beat_t10m_update != t10m_update_seen) {
      t10m_update_seen = beat_t10m_update;
      if (t10m_blocks_accumulated != 0) {
        t10m_reset_count += 1;
      }
      t10m_blocks_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
      aggregation_interval_pending.bit(3) = 0;
#endif
      t10m_target_sample = ctx_t10m_target;
      t10m_target_valid =
          ctx_target_controls.bit(AGG_CONTEXT_TARGET_VALID_BIT);
      basic_utc_target_sample = ctx_t10m_target;
      basic_utc_target_valid = t10m_target_valid;
      // A UTC correction supersedes an unfinished synchronization attempt,
      // but the old authoritative window is still allowed to complete.
      basic_shadow_active = 0;
      if (a3s_shadow_active == 1) {
        if (a3s_blocks_accumulated_slot[a3s_shadow_slot] != 0) {
          a3s_reset_count += 1;
        }
        a3s_blocks_accumulated_slot[a3s_shadow_slot] = 0;
        a3s_shadow_active = 0;
      }
      a3s_sync_seed_pending = 0;
      // Linux normally programs the next UTC mark while capture is already
      // inside the interval, so the first emitted result is explicitly
      // partial.  The following auto-advanced intervals are complete.
      t10m_contaminated = 1;
      // A new UTC mapping terminates the old two-hour chain immediately.
      // Its next seed must be the first complete/aligned ten-minute interval
      // produced under the new mapping.
      if (a2h_intervals_accumulated != 0) {
        a2h_reset_count += 1;
      }
      a2h_intervals_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
      aggregation_interval_pending.bit(4) = 0;
#endif
    }

    const ap_uint<1> beat_apply = ctx_controls.bit(AGG_CONTEXT_APPLY_BIT);
    if (beat_apply != apply_seen) {
      apply_seen = beat_apply;
      active_generation = ctx_cfg_generation;
      active_sample_rate = ctx_cfg_rate;
      active_valid_mask =
          ctx_controls.range(AGG_CONTEXT_MASK_LSB + 7,
                             AGG_CONTEXT_MASK_LSB);
      active_enable = ctx_controls.bit(AGG_CONTEXT_ENABLE_BIT);
      active_dc_remove = ctx_controls.bit(AGG_CONTEXT_DC_REMOVE_BIT);
      arithmetic_overflow = 0;
      basic_active_slot = 0;
      basic_slot = 0;
      basic_cycles_in_block[0] = 0;
      basic_cycles_in_block[1] = 0;
      basic_shadow_active = 0;
      have_expectation = 0;
      disc_pending = 1;
      if (a2h_intervals_accumulated != 0) {
        a2h_reset_count += 1;
      }
      a2h_intervals_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
      aggregation_interval_pending.bit(3) = 0;
      aggregation_interval_pending.bit(4) = 0;
#endif
    }

    if (active_enable == 0) {
      return;
    }

    // Generation boundary: results of another generation never merge.
    if (cycle.generation != active_generation) {
      cycles_in_block = 0;
      basic_shadow_active = 0;
      have_expectation = 0;
      disc_pending = 1;
      return;
    }

    // Continuity: an upstream gap mark, a break in either sequence, or a
    // nominal change discards the partial block; the carrying cycle is a
    // whole valid cycle and starts the next block.
    const bool upstream_gap =
        cycle.status.bit(SCYC_STATUS_FIRST_AFTER_GAP_BIT) == 1;
    const bool sequence_break =
        have_expectation == 1 && (cycle.sequence != expected_result_seq ||
                                  cycle.cycle_sequence != expected_cycle_seq);
    const bool nominal_change =
        cycles_in_block != 0 && cycle.nominal_hz != block_nominal;
    if (upstream_gap || sequence_break || nominal_change) {
      cycles_in_block = 0;
      basic_shadow_active = 0;
      disc_pending = 1;
    }
    expected_result_seq = cycle.sequence + 1;
    expected_cycle_seq = cycle.cycle_sequence + 1;
    have_expectation = 1;

    // The cycles' own arithmetic flags fold into the sticky block flag.
    arithmetic_overflow |= cycle.status.bit(SCYC_STATUS_OVERFLOW_BIT);

    bool start_primary_utc = false;
    bool start_shadow_utc = false;
    if (basic_utc_target_valid == 1 &&
        cycle.first_sample >= basic_utc_target_sample) {
      if (cycles_in_block == 0) {
        start_primary_utc = true;
      } else {
        basic_shadow_slot = basic_slot ^ ap_uint<1>(1);
        basic_cycles_in_block[basic_shadow_slot] = 0;
        basic_shadow_active = 1;
        start_shadow_utc = true;
      }

      // Skip directly to the first future UTC target if a late correction
      // mapped a boundary that is already more than one interval behind.
      const ap_uint<64> interval_samples =
          ap_uint<64>(active_sample_rate) * 600u;
      if (interval_samples == 0) {
        basic_utc_target_valid = 0;
      } else {
        const ap_uint<64> elapsed =
            cycle.first_sample - basic_utc_target_sample;
        const ap_uint<64> intervals = elapsed / interval_samples + 1u;
        basic_utc_target_sample += intervals * interval_samples;
      }
    }

    const bool first_cycle = (cycles_in_block == 0);
    if (first_cycle) {
      block_nominal = cycle.nominal_hz;
      block_cycles_target = met_expected_cycles(cycle.nominal_hz);
      block_first_sample = cycle.first_sample;
      block_sample_count = 0;
      block_mask = 0x7F;
      block_locked_and = 1;
      block_fallback_or = 0;
      block_utc_resynchronized = start_primary_utc;
    }
    block_sample_count += cycle.sample_count;
    block_mask &= cycle.valid_mask;
    block_locked_and &= ctx_controls.bit(AGG_CONTEXT_LOCKED_BIT);
    block_fallback_or |= ctx_controls.bit(AGG_CONTEXT_FALLBACK_BIT);
    const ap_uint<1> cycle_phasor_invalid =
        cycle.status.bit(SCYC_STATUS_PHASOR_INVALID_BIT);
    block_phasor_invalid =
        first_cycle ? cycle_phasor_invalid
                    : ap_uint<1>(block_phasor_invalid | cycle_phasor_invalid);

  merge_lanes:
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
  #pragma HLS PIPELINE off
      const ap_int<128> sum_base = first_cycle ? ap_int<128>(0) : acc_sum[lane];
      acc_sum[lane] = sum_base + cycle.sum[lane];
      const ap_uint<128> square_base =
          first_cycle ? ap_uint<128>(0) : acc_square[lane];
      acc_square[lane] = met_add_square_saturating<128>(
          square_base, cycle.square[lane], arithmetic_overflow);
      const ap_int<64> raw_sum_base =
          first_cycle ? ap_int<64>(0) : acc_raw_sum[lane];
      acc_raw_sum[lane] = raw_sum_base + cycle.raw_sum[lane];
      const ap_uint<96> raw_square_base =
          first_cycle ? ap_uint<96>(0) : acc_raw_square[lane];
      acc_raw_square[lane] = raw_square_base + cycle.raw_square[lane];
      if (first_cycle || cycle.minimum[lane] < acc_minimum[lane]) {
        acc_minimum[lane] = cycle.minimum[lane];
      }
      if (first_cycle || cycle.maximum[lane] > acc_maximum[lane]) {
        acc_maximum[lane] = cycle.maximum[lane];
      }
    }
  merge_power:
    for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
  #pragma HLS PIPELINE off
      const ap_int<128> power_base =
          first_cycle ? ap_int<128>(0) : acc_power[phase];
      acc_power[phase] = met_add_signed_saturating<128>(
          power_base, cycle.power_sum[phase], arithmetic_overflow);
    }
  merge_pairs:
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
  #pragma HLS PIPELINE off
      const ap_uint<128> vll_base =
          first_cycle ? ap_uint<128>(0) : acc_vll_square[pair];
      acc_vll_square[pair] = met_add_square_saturating<128>(
          vll_base, cycle.vll_square[pair], arithmetic_overflow);
    }
  merge_phasor:
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
  #pragma HLS PIPELINE off
      const ap_int<128> re_base =
          first_cycle ? ap_int<128>(0) : acc_phasor_re[lane];
      acc_phasor_re[lane] = met_add_signed_saturating<128>(
          re_base, cycle.phasor_re[lane], arithmetic_overflow);
      const ap_int<128> im_base =
          first_cycle ? ap_int<128>(0) : acc_phasor_im[lane];
      acc_phasor_im[lane] = met_add_signed_saturating<128>(
          im_base, cycle.phasor_im[lane], arithmetic_overflow);
    }

    if (basic_shadow_active == 1) {
      const ap_uint<1> shadow_slot = basic_shadow_slot;
      const int shadow_tier =
          (shadow_slot == 0) ? MET_TIER_BASIC_0 : MET_TIER_BASIC_1;
      const bool shadow_first_cycle =
          (basic_cycles_in_block[shadow_slot] == 0);
      if (shadow_first_cycle) {
        basic_nominal[shadow_slot] = cycle.nominal_hz;
        basic_cycles_target[shadow_slot] =
            met_expected_cycles(cycle.nominal_hz);
        basic_first_sample[shadow_slot] = cycle.first_sample;
        basic_sample_count[shadow_slot] = 0;
        basic_mask[shadow_slot] = 0x7F;
        basic_locked_and[shadow_slot] = 1;
        basic_fallback_or[shadow_slot] = 0;
        basic_utc_resynchronized[shadow_slot] = start_shadow_utc;
      }
      basic_sample_count[shadow_slot] += cycle.sample_count;
      basic_mask[shadow_slot] &= cycle.valid_mask;
      basic_locked_and[shadow_slot] &=
          ctx_controls.bit(AGG_CONTEXT_LOCKED_BIT);
      basic_fallback_or[shadow_slot] |=
          ctx_controls.bit(AGG_CONTEXT_FALLBACK_BIT);
      basic_phasor_invalid[shadow_slot] =
          shadow_first_cycle
              ? cycle_phasor_invalid
              : ap_uint<1>(basic_phasor_invalid[shadow_slot] |
                           cycle_phasor_invalid);

    merge_shadow_lanes:
      for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      #pragma HLS PIPELINE off
        const ap_int<128> sum_base =
            shadow_first_cycle ? ap_int<128>(0)
                               : met_acc_sum[shadow_tier][lane];
        met_acc_sum[shadow_tier][lane] = sum_base + cycle.sum[lane];
        const ap_uint<128> square_base =
            shadow_first_cycle ? ap_uint<128>(0)
                               : met_acc_square[shadow_tier][lane];
        met_acc_square[shadow_tier][lane] =
            met_add_square_saturating<128>(
                square_base, cycle.square[lane], arithmetic_overflow);
        const ap_int<64> raw_sum_base =
            shadow_first_cycle ? ap_int<64>(0)
                               : met_acc_raw_sum[shadow_tier][lane];
        met_acc_raw_sum[shadow_tier][lane] =
            raw_sum_base + cycle.raw_sum[lane];
        const ap_uint<96> raw_square_base =
            shadow_first_cycle ? ap_uint<96>(0)
                               : met_acc_raw_square[shadow_tier][lane];
        met_acc_raw_square[shadow_tier][lane] =
            raw_square_base + cycle.raw_square[lane];
        if (shadow_first_cycle ||
            cycle.minimum[lane] < met_acc_minimum[shadow_tier][lane]) {
          met_acc_minimum[shadow_tier][lane] = cycle.minimum[lane];
        }
        if (shadow_first_cycle ||
            cycle.maximum[lane] > met_acc_maximum[shadow_tier][lane]) {
          met_acc_maximum[shadow_tier][lane] = cycle.maximum[lane];
        }
        const ap_int<128> re_base =
            shadow_first_cycle ? ap_int<128>(0)
                               : met_acc_phasor_re[shadow_tier][lane];
        met_acc_phasor_re[shadow_tier][lane] =
            met_add_signed_saturating<128>(
                re_base, cycle.phasor_re[lane], arithmetic_overflow);
        const ap_int<128> im_base =
            shadow_first_cycle ? ap_int<128>(0)
                               : met_acc_phasor_im[shadow_tier][lane];
        met_acc_phasor_im[shadow_tier][lane] =
            met_add_signed_saturating<128>(
                im_base, cycle.phasor_im[lane], arithmetic_overflow);
      }
    merge_shadow_power:
      for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
      #pragma HLS PIPELINE off
        const ap_int<128> power_base =
            shadow_first_cycle ? ap_int<128>(0)
                               : met_acc_power[shadow_tier][phase];
        met_acc_power[shadow_tier][phase] =
            met_add_signed_saturating<128>(
                power_base, cycle.power_sum[phase], arithmetic_overflow);
      }
    merge_shadow_pairs:
      for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      #pragma HLS PIPELINE off
        const ap_uint<128> vll_base =
            shadow_first_cycle ? ap_uint<128>(0)
                               : met_acc_vll_square[shadow_tier][pair];
        met_acc_vll_square[shadow_tier][pair] =
            met_add_square_saturating<128>(
                vll_base, cycle.vll_square[pair], arithmetic_overflow);
      }
      basic_cycles_in_block[shadow_slot] += 1;
    }

    const ap_uint<8> cycles_now = cycles_in_block + 1;
    if (cycles_now < block_cycles_target) {
      cycles_in_block = cycles_now;
      return;
    }
    cycles_in_block = 0;
    if (basic_shadow_active == 1) {
      basic_active_slot = basic_shadow_slot;
      basic_shadow_active = 0;
    }

    // ---- Shared finalize: ONE call site, up to four passes ---------------
    // pass 0 closes the 10/12-cycle block, pass 1 the 150/180-cycle
    // interval, pass 2 the aligned ten-minute interval, and pass 3 the
    // cascaded two-hour interval. All tiers finalize through the SAME
    // instance because there is only one textual call, whatever the inliner
    // decides. The pass's
    // accumulator set is selected by a ROLLED copy below rather than a
    // parallel mux across ten 896-bit arrays: one 128-bit 2:1 mux per array
    // instead of seven, ~1.3k LUT of selection instead of ~9k.
    //
    // Pass 1 is armed from inside pass 0, because the interval tier can only
    // know it has 15 blocks after the block result exists. The eligibility
    // returns inside the merge therefore end the invocation with the basic
    // quad already emitted, which is exactly the old two-engine behaviour.
    result_mask =
        (active_valid_mask & block_mask) & ap_uint<8>(0x7F);
    count_now = block_sample_count;
    sequence += 1;
    pass_armed = 1;
  }


  ap_int<128>  fin_sum[MET_ACTIVE_CHANNELS];
  ap_uint<128> fin_square[MET_ACTIVE_CHANNELS];
  ap_int<64>   fin_raw_sum[MET_ACTIVE_CHANNELS];
  ap_uint<96>  fin_raw_square[MET_ACTIVE_CHANNELS];
  ap_int<64>   fin_minimum[MET_ACTIVE_CHANNELS];
  ap_int<64>   fin_maximum[MET_ACTIVE_CHANNELS];
  ap_uint<128> fin_vll_square[MET_VLL_PAIRS];
  ap_int<128>  fin_power[MET_POWER_PHASES];
  ap_int<128>  fin_phasor_re[MET_ACTIVE_CHANNELS];
  ap_int<128>  fin_phasor_im[MET_ACTIVE_CHANNELS];
  met_finalize_scratch_t fin_out;
  record_image_t record_image;
#pragma HLS BIND_STORAGE variable=fin_out type=ram_s2p impl=bram
#pragma HLS BIND_STORAGE variable=record_image.word type=ram_s2p impl=bram
  // The selected slot's metadata is static because pass 1 is deferred to the
  // next invocation.  a3s_finalize_slot selects the immutable completed image
  // while the synchronized slot continues accumulating.

finalize_passes:
  for (int pass = 0;
       pass < (MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS ? 6 : 4); ++pass) {
#pragma HLS PIPELINE off
    if (pass_armed.bit(pass) == 0) {
      continue;
    }
    ap_uint<32> fin_count;
    ap_uint<1>  fin_dc;
    ap_uint<8>  fin_mask;
    ap_uint<1>  fin_ovf;
    const bool fin_t10m =
        (pass == 2)
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
        || (pass == 4)
#endif
        ;
    const ap_uint<3> fin_tier =
        (pass == 0) ? ap_uint<3>(BASIC_TIER(basic_slot))
                    : ((pass == 1)
                           ? ap_uint<3>(A3S_TIER(a3s_slot))
                           : (fin_t10m
                                  ? ap_uint<3>(MET_TIER_TEN_MINUTE)
                                  : ap_uint<3>(MET_TIER_TWO_HOUR)));
    if (pass == 0) {
      fin_count = count_now;
      fin_dc = active_dc_remove;
      fin_mask = result_mask;
      fin_ovf = arithmetic_overflow;
    } else if (pass == 1) {
      fin_count = a3s_agg_total_samples;
      fin_dc = a3s_agg_dc_remove;
      fin_mask = a3s_mask_and & ap_uint<8>(0x7F);
      fin_ovf = a3s_arithmetic_flag;
    } else if (fin_t10m) {
      fin_count = t10m_total_samples;
      fin_dc = t10m_dc_remove;
      fin_mask = t10m_mask_and & ap_uint<8>(0x7F);
      fin_ovf = t10m_arithmetic_flag;
    } else {
      fin_count = a2h_total_samples;
      fin_dc = a2h_dc_remove;
      fin_mask = a2h_mask_and & ap_uint<8>(0x7F);
      fin_ovf = a2h_arithmetic_flag;
    }
  fin_select_lanes:
    for (int i = 0; i < MET_ACTIVE_CHANNELS; ++i) {
#pragma HLS PIPELINE off
      fin_sum[i] = met_acc_sum[fin_tier][i];
      fin_square[i] = met_acc_square[fin_tier][i];
      fin_raw_sum[i] = met_acc_raw_sum[fin_tier][i];
      fin_raw_square[i] = met_acc_raw_square[fin_tier][i];
      fin_minimum[i] = met_acc_minimum[fin_tier][i];
      fin_maximum[i] = met_acc_maximum[fin_tier][i];
      fin_phasor_re[i] = met_acc_phasor_re[fin_tier][i];
      fin_phasor_im[i] = met_acc_phasor_im[fin_tier][i];
    }
  fin_select_pairs:
    for (int p = 0; p < MET_VLL_PAIRS; ++p) {
#pragma HLS PIPELINE off
      fin_vll_square[p] = met_acc_vll_square[fin_tier][p];
    }
  fin_select_power:
    for (int p = 0; p < MET_POWER_PHASES; ++p) {
#pragma HLS PIPELINE off
      fin_power[p] = met_acc_power[fin_tier][p];
    }

    met_finalize_interval(fin_sum, fin_square, fin_raw_sum, fin_raw_square,
                          fin_minimum, fin_maximum, fin_vll_square, fin_power,
                          fin_phasor_re, fin_phasor_im, fin_count, fin_dc,
                          fin_mask, fin_out, fin_ovf);

    // The finalize ORs into the flag it is given, so seed-and-write-back
    // keeps each tier's sticky arithmetic bit exactly as it was.
    if (pass == 0) {
      arithmetic_overflow = fin_ovf;
    } else if (pass == 1) {
      a3s_arithmetic_flag = fin_ovf;
    } else if (fin_t10m) {
      t10m_arithmetic_flag = fin_ovf;
    } else {
      a2h_arithmetic_flag = fin_ovf;
    }

    if (pass == 0) {

    const ap_uint<1> first_block = disc_pending;
    disc_pending = 0;
    const ap_uint<32> status =
        ap_uint<32>(arithmetic_overflow) | (ap_uint<32>(first_block) << 2);
    ap_uint<3> flags = 0;
    flags[MET_FLAG_LOCKED] = block_locked_and;
    flags[MET_FLAG_FALLBACK] = block_fallback_or;
    flags[MET_FLAG_FIRST_BLOCK] = first_block;

    // Local Basic result: provenance plus merge-safe accumulators. The longer
    // tiers merge this value by pure addition and never re-derive it.
    agg_block_result_t result;
    result.sequence = sequence;
    result.generation = active_generation;
    result.first_sample = block_first_sample;
    result.last_sample = cycle.last_sample;
    result.sample_count = count_now;
    result.sample_rate_hz = active_sample_rate;
    result.nominal_hz = block_nominal;
    result.valid_mask = result_mask;
    result.flags = flags;
    result.cycle_count = block_cycles_target;
    result.status =
        status | (ap_uint<32>(block_phasor_invalid) << PHASOR_STATUS_INVALID_BIT);
    result.frequency_millihz = cycle.frequency_millihz;
    result.frequency_valid = cycle.frequency_valid;
    result.apply_toggle = apply_seen;
    result.dc_remove = active_dc_remove;
    result.utc_resynchronized = block_utc_resynchronized;
  result_accumulators:
    for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
  #pragma HLS PIPELINE off
      result.sum[lane] = acc_sum[lane];
      result.square[lane] = acc_square[lane];
      result.raw_sum[lane] = acc_raw_sum[lane];
      result.raw_square[lane] = acc_raw_square[lane];
      result.minimum[lane] = acc_minimum[lane];
      result.maximum[lane] = acc_maximum[lane];
      result.phasor_re[lane] = acc_phasor_re[lane];
      result.phasor_im[lane] = acc_phasor_im[lane];
    }
  result_pairs:
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
  #pragma HLS PIPELINE off
      result.vll_square[pair] = acc_vll_square[pair];
    }
  result_power:
    for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
  #pragma HLS PIPELINE off
      result.power_sum[phase] = acc_power[phase];
    }

    // BASIC-v4 record (MTR1-v3 interior plus the documented additions).
    clear_record(record_image);
    fill_envelope(record_image, sequence, active_generation, active_sample_rate,
                  count_now, result_mask, status, block_first_sample);
    record_image.word[MTR1_TIMING_WORD] =
        (ap_uint<32>(block_nominal) << MTR1_TIMING_NOMINAL_LSB) |
        (ap_uint<32>(block_cycles_target) << MTR1_TIMING_CYCLES_LSB) |
        (ap_uint<32>(flags) << MTR1_TIMING_FLAGS_LSB) |
        (ap_uint<32>(block_utc_resynchronized)
         << MTR1_TIMING_UTC_RESYNCHRONIZED_BIT);
    record_image.word[BASIC_LAST_SAMPLE_LOW_WORD] =
        cycle.last_sample.range(31, 0);
    record_image.word[BASIC_LAST_SAMPLE_HIGH_WORD] =
        cycle.last_sample.range(63, 32);
  record_lanes:
    for (int lane = 0; lane < MET_CHANNEL_LANES; ++lane) {
  #pragma HLS PIPELINE off
      if (lane < MET_ACTIVE_CHANNELS) {
        const ap_int<64> mean_units =
            met_fin_mean_q16(fin_out, lane) >> 16;  // arithmetic
        const ap_uint<64> rms_units = met_fin_rms_q16(fin_out, lane) >> 16;
        const int base = MTR1_CH_BASE_WORD + lane * MTR1_CH_STRIDE_WORDS;
        record_image.word[base + MTR1_CH_MEAN_LOW] =
            ap_uint<64>(mean_units).range(31, 0);
        record_image.word[base + MTR1_CH_MEAN_HIGH] =
            ap_uint<64>(mean_units).range(63, 32);
        record_image.word[base + MTR1_CH_RMS_COUNT] =
            met_fin_rms_count(fin_out, lane);
        record_image.word[base + MTR1_CH_RMS_LOW] = rms_units.range(31, 0);
        record_image.word[base + MTR1_CH_RMS_HIGH] = rms_units.range(63, 32);
      }
    }
  record_pairs:
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
  #pragma HLS PIPELINE off
      record_image.word[BASIC_VLL_BASE_WORD + pair] =
          ap_uint<64>(met_fin_vll_rms(fin_out, pair) >> 16).range(31, 0);
    }
    record_image.word[MTR1_FREQUENCY_VALUE_WORD] = cycle.frequency_millihz;
    record_image.word[MTR1_FREQUENCY_STATUS_WORD] =
        ctx_freq_status;
    record_image.word[MTR1_FREQUENCY_PERIOD_WORD] =
        ctx_freq_period;
    record_image.word[MTR1_FREQUENCY_SEQUENCE_WORD] =
        ctx_freq_seq;
    record_image.word[MTR1_CAPTURE_FRAMES_WORD] =
        ctx_cap_frames;
    record_image.word[MTR1_HEADER_ERRORS_WORD] =
        ctx_cap_hdrerr;
    record_image.word[MTR1_FIFO_OVERFLOWS_WORD] =
        ctx_cap_overflow;
    record_image.word[MTR1_ADC_ALERTS_WORD] =
        ctx_cap_alerts;

    const ap_uint<32> record_timing_word =
        record_image.word[MTR1_TIMING_WORD];
    const ap_uint<32> record_last_sample_low =
        record_image.word[BASIC_LAST_SAMPLE_LOW_WORD];
    const ap_uint<32> record_last_sample_high =
        record_image.word[BASIC_LAST_SAMPLE_HIGH_WORD];
    serialize_record_format(record_image, MREC_FORMAT_BASIC_V4, m_basic);

    // POWER-v1 record on the same stream, describing the same block (same
    // sequence, generation, anchors, status).
    clear_record(record_image);
    fill_envelope(record_image, sequence, active_generation, active_sample_rate,
                  count_now, result_mask, status, block_first_sample);
    record_image.word[MTR1_TIMING_WORD] = record_timing_word;
    record_image.word[BASIC_LAST_SAMPLE_LOW_WORD] = record_last_sample_low;
    record_image.word[BASIC_LAST_SAMPLE_HIGH_WORD] = record_last_sample_high;
    fill_power_payload(fin_out, record_image);
    serialize_record_format(record_image, MREC_FORMAT_POWER_V1, m_basic);

    // PHASOR-v1 record, third on the stream for the same block. Only the
    // PHASOR and UNBAL records carry the block phasor-invalid status bit.
    clear_record(record_image);
    const ap_uint<32> phasor_status =
        status |
        (ap_uint<32>(block_phasor_invalid) << PHASOR_STATUS_INVALID_BIT);
    fill_envelope(record_image, sequence, active_generation, active_sample_rate,
                  count_now, result_mask, phasor_status, block_first_sample);
    record_image.word[MTR1_TIMING_WORD] = record_timing_word;
    record_image.word[BASIC_LAST_SAMPLE_LOW_WORD] = record_last_sample_low;
    record_image.word[BASIC_LAST_SAMPLE_HIGH_WORD] = record_last_sample_high;
    fill_phasor_payload(fin_out, record_image);
    serialize_record_format(record_image, MREC_FORMAT_PHASOR_V2, m_basic);

    // UNBALANCE-v1 record, fourth on the stream (M10).
    clear_record(record_image);
    fill_envelope(record_image, sequence, active_generation, active_sample_rate,
                  count_now, result_mask, phasor_status, block_first_sample);
    record_image.word[MTR1_TIMING_WORD] = record_timing_word;
    record_image.word[BASIC_LAST_SAMPLE_LOW_WORD] = record_last_sample_low;
    record_image.word[BASIC_LAST_SAMPLE_HIGH_WORD] = record_last_sample_high;
    fill_unbal_payload(fin_out, record_image);
    serialize_record_format(record_image, MREC_FORMAT_UNBAL_V2, m_basic);

    // ==== Clock-aligned ten-minute interval tier (M13) ===================
    // This consumes the BASIC result directly, in parallel with the 3 s
    // tier below.  It never consumes the 3 s record.  A Linux-provided
    // sample target maps the next UTC ten-minute mark into the PL sample
    // counter; the first complete basic block ending at/after that target
    // closes the interval.  Overshoot is recorded rather than hidden.
    const bool t10m_nominal_known =
        (result.nominal_hz == 50) || (result.nominal_hz == 60);
    const bool t10m_input_eligible =
        result.flags.bit(MET_FLAG_LOCKED) == 1 &&
        result.flags.bit(MET_FLAG_FALLBACK) == 0 &&
        result.flags.bit(MET_FLAG_FIRST_BLOCK) == 0 && t10m_nominal_known &&
        result.cycle_count == met_expected_cycles(result.nominal_hz);

    if (t10m_target_valid == 0) {
      if (t10m_blocks_accumulated != 0) {
        t10m_reset_count += 1;
      }
      t10m_blocks_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
      aggregation_interval_pending.bit(3) = 0;
#endif
      t10m_contaminated = 1;
    } else if (!t10m_input_eligible) {
      t10m_ineligible_count += 1;
      if (t10m_blocks_accumulated != 0) {
        t10m_reset_count += 1;
      }
      t10m_blocks_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
      aggregation_interval_pending.bit(3) = 0;
#endif
      t10m_contaminated = 1;
    } else {
      bool t10m_seed = (t10m_blocks_accumulated == 0);
      if (!t10m_seed) {
        if (result.generation != t10m_generation ||
            result.nominal_hz != t10m_nominal ||
            result.sample_rate_hz != t10m_sample_rate ||
            result.dc_remove != t10m_dc_remove) {
          t10m_reset_count += 1;
          t10m_contaminated = 1;
          t10m_seed = true;
        } else if (result.sequence != t10m_expected_next_seq ||
                   result.first_sample != t10m_expected_next_first) {
          t10m_continuity_count += 1;
          t10m_reset_count += 1;
          t10m_contaminated = 1;
          t10m_seed = true;
        }
      }

      if (t10m_seed) {
        t10m_generation = result.generation;
        t10m_nominal = result.nominal_hz;
        t10m_sample_rate = result.sample_rate_hz;
        t10m_dc_remove = result.dc_remove;
        t10m_first_sample = result.first_sample;
        t10m_first_seq = result.sequence;
        t10m_total_samples = result.sample_count;
        t10m_total_cycles = result.cycle_count;
        t10m_mask_and = result.valid_mask;
        t10m_arithmetic_flag =
            result.status.bit(MREC_STATUS_ARITHMETIC_BIT);
        t10m_phasor_invalid_or =
            result.status.bit(PHASOR_STATUS_INVALID_BIT);
        t10m_blocks_accumulated = 1;
      } else {
        t10m_total_samples += result.sample_count;
        t10m_total_cycles += result.cycle_count;
        t10m_mask_and &= result.valid_mask;
        t10m_arithmetic_flag |=
            result.status.bit(MREC_STATUS_ARITHMETIC_BIT);
        t10m_phasor_invalid_or |=
            result.status.bit(PHASOR_STATUS_INVALID_BIT);
        t10m_blocks_accumulated += 1;
      }
      t10m_last_sample = result.last_sample;
      t10m_last_seq = result.sequence;
      t10m_expected_next_seq = result.sequence + 1;
      t10m_expected_next_first = result.first_sample + result.sample_count;

    t10m_merge_lanes:
      for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
      #pragma HLS PIPELINE off
        const ap_int<128> sum_base =
            t10m_seed ? ap_int<128>(0) : t10m_acc_sum[lane];
        t10m_acc_sum[lane] = sum_base + result.sum[lane];
        const ap_uint<128> square_base =
            t10m_seed ? ap_uint<128>(0) : t10m_acc_square[lane];
        t10m_acc_square[lane] = met_add_square_saturating<128>(
            square_base, result.square[lane], t10m_arithmetic_flag);
        const ap_int<64> raw_sum_base =
            t10m_seed ? ap_int<64>(0) : t10m_acc_raw_sum[lane];
        t10m_acc_raw_sum[lane] = raw_sum_base + result.raw_sum[lane];
        const ap_uint<96> raw_square_base =
            t10m_seed ? ap_uint<96>(0) : t10m_acc_raw_square[lane];
        t10m_acc_raw_square[lane] = raw_square_base + result.raw_square[lane];
        if (t10m_seed || result.minimum[lane] < t10m_acc_minimum[lane]) {
          t10m_acc_minimum[lane] = result.minimum[lane];
        }
        if (t10m_seed || result.maximum[lane] > t10m_acc_maximum[lane]) {
          t10m_acc_maximum[lane] = result.maximum[lane];
        }
        const ap_int<128> re_base =
            t10m_seed ? ap_int<128>(0) : t10m_acc_phasor_re[lane];
        t10m_acc_phasor_re[lane] = met_add_signed_saturating<128>(
            re_base, result.phasor_re[lane], t10m_arithmetic_flag);
        const ap_int<128> im_base =
            t10m_seed ? ap_int<128>(0) : t10m_acc_phasor_im[lane];
        t10m_acc_phasor_im[lane] = met_add_signed_saturating<128>(
            im_base, result.phasor_im[lane], t10m_arithmetic_flag);
      }
    t10m_merge_power:
      for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
      #pragma HLS PIPELINE off
        const ap_int<128> power_base =
            t10m_seed ? ap_int<128>(0) : t10m_acc_power[phase];
        t10m_acc_power[phase] = met_add_signed_saturating<128>(
            power_base, result.power_sum[phase], t10m_arithmetic_flag);
      }
    t10m_merge_pairs:
      for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      #pragma HLS PIPELINE off
        const ap_uint<128> vll_base =
            t10m_seed ? ap_uint<128>(0) : t10m_acc_vll_square[pair];
        t10m_acc_vll_square[pair] = met_add_square_saturating<128>(
            vll_base, result.vll_square[pair], t10m_arithmetic_flag);
      }

      if (result.last_sample >= t10m_target_sample) {
        t10m_closed_target = t10m_target_sample;
        const ap_uint<64> overshoot = result.last_sample - t10m_target_sample;
        t10m_overshoot_samples =
            (overshoot > ap_uint<64>(0xFFFFFFFFu))
                ? ap_uint<32>(0xFFFFFFFFu)
                : ap_uint<32>(overshoot);
        // Continue autonomously at exact 600-second sample intervals. Linux
        // sends another UPDATE after a time/source/rate discontinuity.
        t10m_target_sample += ap_uint<64>(result.sample_rate_hz) * 600u;
        aggregation_interval_pending.bit(1) = 1;
      }
    }

    // ==== 150/180-cycle (3 s) interval tier ================================
    // Was Agg150_180CycleEngine. Its input is no longer an AXIS beat: the
    // block result is the local `result` built above, so the 7072-bit
    // inter-tier interface, its FIFO, and its shim are gone. Every rule
    // below is the retired engine's verbatim, statics prefixed a3s_ to
    // share one scope with the block tier.
    // A configuration APPLY between beats terminates any partially
    // accumulated aggregate before this beat is considered (Mtr2 rule).
    if (result.apply_toggle != a3s_apply_seen) {
      a3s_apply_seen = result.apply_toggle;
      if (a3s_blocks_accumulated_slot[0] != 0 ||
          a3s_blocks_accumulated_slot[1] != 0) {
        a3s_reset_count += 1;
      }
      a3s_blocks_accumulated_slot[0] = 0;
      a3s_blocks_accumulated_slot[1] = 0;
      a3s_shadow_active = 0;
      a3s_sync_seed_pending = 0;
      a3s_slot = a3s_active_slot;
    }

    // Eligibility: identical predicate to the retired Mtr2 engine and the
    // APU's class_a_aggregation_eligible() rule.
    const bool a3s_nominal_known = (result.nominal_hz == 50) || (result.nominal_hz == 60);
    const bool a3s_input_eligible =
        result.flags.bit(MET_FLAG_LOCKED) == 1 &&
        result.flags.bit(MET_FLAG_FALLBACK) == 0 &&
        result.flags.bit(MET_FLAG_FIRST_BLOCK) == 0 && a3s_nominal_known &&
        result.cycle_count == met_expected_cycles(result.nominal_hz);

    if (!a3s_input_eligible) {
      // An ineligible block invalidates the running aggregate and never
      // seeds a new one: the 150/180-cycle interval must stay contiguous.
      a3s_ineligible_count += 1;
      if (a3s_blocks_accumulated_slot[0] != 0 ||
          a3s_blocks_accumulated_slot[1] != 0) {
        a3s_reset_count += 1;
      }
      a3s_blocks_accumulated_slot[0] = 0;
      a3s_blocks_accumulated_slot[1] = 0;
      a3s_shadow_active = 0;
      a3s_sync_seed_pending = result.utc_resynchronized;
      return;
    }

    a3s_slot = a3s_active_slot;
    bool a3s_seed = (a3s_blocks_accumulated == 0);
    if (!a3s_seed) {
      if (result.generation != a3s_agg_generation || result.nominal_hz != a3s_agg_nominal ||
          result.sample_rate_hz != a3s_agg_sample_rate ||
          result.dc_remove != a3s_agg_dc_remove) {
        // Generation, nominal, sample-rate, or dc_remove change: discard
        // the partial aggregate; this block seeds the next one.
        a3s_reset_count += 1;
        a3s_blocks_accumulated_slot[0] = 0;
        a3s_blocks_accumulated_slot[1] = 0;
        a3s_shadow_active = 0;
        a3s_seed = true;
      } else {
        const bool sequence_continuous =
            result.sequence == a3s_expected_next_seq;
        const bool sample_continuous =
            result.first_sample == a3s_expected_next_first;
        const bool marked_overlap =
            result.utc_resynchronized == 1 && sequence_continuous &&
            result.first_sample <= a3s_agg_last_sample &&
            result.last_sample > a3s_agg_last_sample;
        if (!sequence_continuous || (!sample_continuous && !marked_overlap)) {
          // Lost/reordered block or an unmarked sample-domain break cannot
          // enter either interval.  The one marked UTC overlap is deliberate.
          a3s_continuity_count += 1;
          a3s_reset_count += 1;
          a3s_blocks_accumulated_slot[0] = 0;
          a3s_blocks_accumulated_slot[1] = 0;
          a3s_shadow_active = 0;
          a3s_seed = true;
        }
      }
    }

    const bool start_synchronized_shadow =
        result.utc_resynchronized == 1 && !a3s_seed;
    if (start_synchronized_shadow) {
      if (a3s_shadow_active == 1 &&
          a3s_blocks_accumulated_slot[a3s_shadow_slot] != 0) {
        a3s_reset_count += 1;
      }
      a3s_shadow_slot = a3s_slot ^ ap_uint<1>(1);
      a3s_blocks_accumulated_slot[a3s_shadow_slot] = 0;
      a3s_shadow_active = 1;
      a3s_utc_overlap = 1;
    }
    if (result.utc_resynchronized == 1) {
      a3s_sync_seed_pending = 1;
    }

    auto merge_a3s_slot = [&](const ap_uint<1> slot, const bool seed,
                              const bool synchronized_seed) {
      const int tier =
          (slot == 0) ? MET_TIER_AGGREGATE_0 : MET_TIER_AGGREGATE_1;
      if (seed) {
        a3s_agg_generation_slot[slot] = result.generation;
        a3s_agg_nominal_slot[slot] = result.nominal_hz;
        a3s_agg_sample_rate_slot[slot] = result.sample_rate_hz;
        a3s_agg_dc_remove_slot[slot] = result.dc_remove;
        a3s_agg_first_sample_slot[slot] = result.first_sample;
        a3s_agg_first_seq_slot[slot] = result.sequence;
        a3s_agg_total_samples_slot[slot] = result.sample_count;
        a3s_agg_total_cycles_slot[slot] = result.cycle_count;
        a3s_mask_and_slot[slot] = result.valid_mask;
        a3s_freq_sum_slot[slot] = result.frequency_millihz;
        a3s_freq_all_valid_slot[slot] = result.frequency_valid;
        a3s_arithmetic_flag_slot[slot] =
            result.status.bit(MREC_STATUS_ARITHMETIC_BIT);
        a3s_phasor_invalid_or_slot[slot] =
            result.status.bit(PHASOR_STATUS_INVALID_BIT);
        a3s_utc_overlap_slot[slot] = 0;
        a3s_utc_resynchronized_slot[slot] = synchronized_seed;
        a3s_blocks_accumulated_slot[slot] = 1;
      } else {
        a3s_agg_total_samples_slot[slot] += result.sample_count;
        a3s_agg_total_cycles_slot[slot] += result.cycle_count;
        a3s_mask_and_slot[slot] &= result.valid_mask;
        a3s_freq_sum_slot[slot] += result.frequency_millihz;
        a3s_freq_all_valid_slot[slot] &= result.frequency_valid;
        a3s_arithmetic_flag_slot[slot] |=
            result.status.bit(MREC_STATUS_ARITHMETIC_BIT);
        a3s_phasor_invalid_or_slot[slot] |=
            result.status.bit(PHASOR_STATUS_INVALID_BIT);
        a3s_blocks_accumulated_slot[slot] += 1;
      }
      a3s_agg_last_sample_slot[slot] = result.last_sample;
      a3s_agg_last_seq_slot[slot] = result.sequence;
      a3s_expected_next_seq_slot[slot] = result.sequence + 1;
      a3s_expected_next_first_slot[slot] =
          result.first_sample + result.sample_count;

      for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
        const ap_int<128> sum_base =
            seed ? ap_int<128>(0) : met_acc_sum[tier][lane];
        met_acc_sum[tier][lane] = sum_base + result.sum[lane];
        const ap_uint<128> square_base =
            seed ? ap_uint<128>(0) : met_acc_square[tier][lane];
        met_acc_square[tier][lane] = met_add_square_saturating<128>(
            square_base, result.square[lane],
            a3s_arithmetic_flag_slot[slot]);
        const ap_int<64> raw_sum_base =
            seed ? ap_int<64>(0) : met_acc_raw_sum[tier][lane];
        met_acc_raw_sum[tier][lane] = raw_sum_base + result.raw_sum[lane];
        const ap_uint<96> raw_square_base =
            seed ? ap_uint<96>(0) : met_acc_raw_square[tier][lane];
        met_acc_raw_square[tier][lane] =
            raw_square_base + result.raw_square[lane];
        if (seed || result.minimum[lane] < met_acc_minimum[tier][lane]) {
          met_acc_minimum[tier][lane] = result.minimum[lane];
        }
        if (seed || result.maximum[lane] > met_acc_maximum[tier][lane]) {
          met_acc_maximum[tier][lane] = result.maximum[lane];
        }
        const ap_int<128> re_base =
            seed ? ap_int<128>(0) : met_acc_phasor_re[tier][lane];
        met_acc_phasor_re[tier][lane] = met_add_signed_saturating<128>(
            re_base, result.phasor_re[lane],
            a3s_arithmetic_flag_slot[slot]);
        const ap_int<128> im_base =
            seed ? ap_int<128>(0) : met_acc_phasor_im[tier][lane];
        met_acc_phasor_im[tier][lane] = met_add_signed_saturating<128>(
            im_base, result.phasor_im[lane],
            a3s_arithmetic_flag_slot[slot]);
      }
      for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
        const ap_int<128> power_base =
            seed ? ap_int<128>(0) : met_acc_power[tier][phase];
        met_acc_power[tier][phase] = met_add_signed_saturating<128>(
            power_base, result.power_sum[phase],
            a3s_arithmetic_flag_slot[slot]);
      }
      for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
        const ap_uint<128> vll_base =
            seed ? ap_uint<128>(0) : met_acc_vll_square[tier][pair];
        met_acc_vll_square[tier][pair] = met_add_square_saturating<128>(
            vll_base, result.vll_square[pair],
            a3s_arithmetic_flag_slot[slot]);
      }
    };

    const bool active_synchronized_seed =
        a3s_seed && a3s_sync_seed_pending == 1;
    merge_a3s_slot(a3s_slot, a3s_seed, active_synchronized_seed);
    if (start_synchronized_shadow) {
      merge_a3s_slot(a3s_shadow_slot, true, true);
    } else if (a3s_shadow_active == 1 && a3s_shadow_slot != a3s_slot) {
      merge_a3s_slot(a3s_shadow_slot, false, false);
    }
    if (active_synchronized_seed || start_synchronized_shadow) {
      a3s_sync_seed_pending = 0;
    }

    if (a3s_blocks_accumulated != MET_BASIC_BLOCKS_PER_AGGREGATE) {
      return;
    }

    // Fifteenth eligible block: finalize the whole interval (the SHARED
    // arithmetic — one definition for both tiers) and emit the record quad.
      // Arming pass 1: reaching here means the merge accepted this block
      // and it was the fifteenth, so the interval closes on this beat too.
      // Do not run pass 1 now -- defer it to the next invocation.
      a3s_finalize_slot = a3s_slot;
      aggregation_interval_pending.bit(0) = 1;
      if (a3s_shadow_active == 1 && a3s_shadow_slot != a3s_slot) {
        a3s_active_slot = a3s_shadow_slot;
        a3s_shadow_active = 0;
      }
    } else if (pass == 1) {
      // Derived from the interval statics the merge just updated; these
      // used to sit immediately above the second finalize call.
      const ap_uint<8> a3s_result_mask = a3s_mask_and & ap_uint<8>(0x7F);
      const ap_uint<32> a3s_count_now = a3s_agg_total_samples;
      (void)a3s_result_mask;
    const ap_uint<36> a3s_freq_mean =
        floor_div_const<36, MET_BASIC_BLOCKS_PER_AGGREGATE>(a3s_freq_sum);

    a3s_out_sequence += 1;
    a3s_blocks_accumulated = 0;

    const ap_uint<32> a3s_shape_word =
        (ap_uint<32>(MET_BASIC_BLOCKS_PER_AGGREGATE) << MTR2_SHAPE_BLOCKS_LSB) |
        (ap_uint<32>(a3s_agg_nominal) << MTR2_SHAPE_NOMINAL_LSB) |
        (ap_uint<32>(a3s_agg_total_cycles) << MTR2_SHAPE_CYCLES_LSB);

    // ---- AGG-v3: the aggregate fundamental record (MTR2 interior + the
    // ---- documented additions; a3s_status keeps the MTR2 bit semantics). ----
    clear_record(record_image);
    const ap_uint<32> agg_status =
        (ap_uint<32>(a3s_arithmetic_flag) << MREC_STATUS_ARITHMETIC_BIT) |
        (ap_uint<32>(1) << MTR2_STATUS_COMPLETE_BIT) |
        (ap_uint<32>(a3s_freq_all_valid) << MTR2_STATUS_FREQUENCY_BIT) |
        (ap_uint<32>(a3s_utc_overlap) << MTR2_STATUS_UTC_OVERLAP_BIT) |
        (ap_uint<32>(a3s_utc_resynchronized)
         << MTR2_STATUS_UTC_RESYNCHRONIZED_BIT);
    fill_envelope(record_image, a3s_out_sequence, a3s_agg_generation,
                  a3s_agg_sample_rate,
                  a3s_count_now, a3s_result_mask, agg_status, a3s_agg_first_sample);
    record_image.word[MTR2_SHAPE_WORD] = a3s_shape_word;
    record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a3s_agg_first_seq;
    record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a3s_agg_last_seq;
  agg_record_lanes:
    for (int lane = 0; lane < MET_CHANNEL_LANES; ++lane) {
  #pragma HLS PIPELINE off
      if (lane < MET_ACTIVE_CHANNELS) {
        const ap_uint<64> rms_units = met_fin_rms_q16(fin_out, lane) >> 16;
        const int base = MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS;
        record_image.word[base + 0] = rms_units.range(31, 0);
        record_image.word[base + 1] = rms_units.range(63, 32);
      }
    }
    record_image.word[MTR2_FREQUENCY_WORD] =
        (a3s_freq_all_valid == 1) ? ap_uint<32>(a3s_freq_mean.range(31, 0))
                              : ap_uint<32>(0);
    record_image.word[MTR2_RESET_COUNT_WORD] = a3s_reset_count;
    record_image.word[MTR2_INELIGIBLE_COUNT_WORD] = a3s_ineligible_count;
    record_image.word[MTR2_CONTINUITY_COUNT_WORD] = a3s_continuity_count;
    record_image.word[AGG_LAST_SAMPLE_LOW_WORD] =
        a3s_agg_last_sample.range(31, 0);
    record_image.word[AGG_LAST_SAMPLE_HIGH_WORD] =
        a3s_agg_last_sample.range(63, 32);
  agg_record_pairs:
    for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
  #pragma HLS PIPELINE off
      record_image.word[AGG_VLL_BASE_WORD + pair] =
          ap_uint<64>(met_fin_vll_rms(fin_out, pair) >> 16).range(31, 0);
    }
    serialize_record_format(record_image, MREC_FORMAT_AGG_V3, m_agg);

    // Sibling a3s_status: common arithmetic bit, plus phasor-invalid (bit 1)
    // on the phasor-domain records — the basic-period siblings' semantics.
    const ap_uint<32> sibling_status =
        (ap_uint<32>(a3s_arithmetic_flag) << MREC_STATUS_ARITHMETIC_BIT) |
        (ap_uint<32>(a3s_utc_overlap) << MTR2_STATUS_UTC_OVERLAP_BIT) |
        (ap_uint<32>(a3s_utc_resynchronized)
         << MTR2_STATUS_UTC_RESYNCHRONIZED_BIT);
    const ap_uint<32> a3s_phasor_status =
        sibling_status |
        (ap_uint<32>(a3s_phasor_invalid_or) << PHASOR_STATUS_INVALID_BIT);

    // ---- AGG-POWER: payload map identical to POWER-v1. ------------------
    clear_record(record_image);
    fill_envelope(record_image, a3s_out_sequence, a3s_agg_generation,
                  a3s_agg_sample_rate,
                  a3s_count_now, a3s_result_mask, sibling_status, a3s_agg_first_sample);
    record_image.word[MTR2_SHAPE_WORD] = a3s_shape_word;
    record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a3s_agg_first_seq;
    record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a3s_agg_last_seq;
    fill_power_payload(fin_out, record_image);
    serialize_record_format(record_image, MREC_FORMAT_AGG_POWER_V1, m_agg);

    // ---- AGG-PHASOR: payload map identical to PHASOR-v1. ----------------
    clear_record(record_image);
    fill_envelope(record_image, a3s_out_sequence, a3s_agg_generation,
                  a3s_agg_sample_rate,
                  a3s_count_now, a3s_result_mask, a3s_phasor_status, a3s_agg_first_sample);
    record_image.word[MTR2_SHAPE_WORD] = a3s_shape_word;
    record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a3s_agg_first_seq;
    record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a3s_agg_last_seq;
    fill_phasor_payload(fin_out, record_image);
    serialize_record_format(record_image, MREC_FORMAT_AGG_PHASOR_V2, m_agg);

    // ---- AGG-UNBAL: payload map identical to UNBAL-v1. ------------------
    clear_record(record_image);
    fill_envelope(record_image, a3s_out_sequence, a3s_agg_generation,
                  a3s_agg_sample_rate,
                  a3s_count_now, a3s_result_mask, a3s_phasor_status, a3s_agg_first_sample);
    record_image.word[MTR2_SHAPE_WORD] = a3s_shape_word;
    record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a3s_agg_first_seq;
    record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a3s_agg_last_seq;
    fill_unbal_payload(fin_out, record_image);
    serialize_record_format(record_image, MREC_FORMAT_AGG_UNBAL_V2, m_agg);

#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
    // The 150/180-cycle cadence is the publication cadence for the open
    // ten-minute view.  Never queue a preview when this same Basic block
    // closed the authoritative ten-minute interval: pass 2 must consume the
    // immutable accumulator image before it is reset.
    if (t10m_blocks_accumulated != 0 &&
        aggregation_interval_pending.bit(1) == 0) {
      aggregation_interval_pending.bit(3) = 1;
    }
#endif
    } else if (pass == 2) {
      // ---- TEN-MINUTE-v1: one quad for the interval which closed at the
      // first complete basic block ending at/after the programmed UTC
      // sample target.  Frequency is intentionally absent; the Class-A
      // 10-second frequency path is a separate direct-cycle producer.
      const ap_uint<8> t10m_result_mask =
          t10m_mask_and & ap_uint<8>(0x7F);
      const ap_uint<32> t10m_count_now = t10m_total_samples;
      const ap_uint<1> t10m_aligned =
          t10m_target_valid & ap_uint<1>(!t10m_contaminated);
      ap_uint<8> t10m_flags = 0;
      t10m_flags.bit(TEN_MINUTE_FLAG_CONTAMINATED_BIT) = t10m_contaminated;
      t10m_flags.bit(TEN_MINUTE_FLAG_ALIGNED_BIT) = t10m_aligned;

      t10m_out_sequence += 1;
      const ap_uint<32> t10m_shape_word =
          (ap_uint<32>(t10m_blocks_accumulated)
           << TEN_MINUTE_SHAPE_BLOCKS_LSB) |
          (ap_uint<32>(t10m_nominal) << TEN_MINUTE_SHAPE_NOMINAL_LSB) |
          (ap_uint<32>(t10m_flags) << TEN_MINUTE_SHAPE_FLAGS_LSB);
      const ap_uint<32> t10m_status =
          (ap_uint<32>(t10m_arithmetic_flag)
           << MREC_STATUS_ARITHMETIC_BIT) |
          (ap_uint<32>(1) << TEN_MINUTE_STATUS_COMPLETE_BIT) |
          (ap_uint<32>(t10m_aligned)
           << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) |
          (ap_uint<32>(t10m_contaminated)
           << TEN_MINUTE_STATUS_CONTAMINATED_BIT) |
          (ap_uint<32>(t10m_target_valid)
           << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT);

      // ==== Cascaded two-hour interval tier (M14) ========================
      // The two-hour result is the only tier allowed to consume another
      // aggregate.  It merges the merge-safe accumulator state behind each
      // complete, aligned ten-minute record; finalized RMS/power values are
      // never averaged or re-derived.  This preserves the exact arithmetic
      // of one continuous two-hour sample interval while sharing the one
      // finalizer above.
      const bool a2h_input_eligible =
          t10m_aligned == 1 && t10m_target_valid == 1 &&
          t10m_contaminated == 0 && t10m_blocks_accumulated != 0 &&
          ((t10m_nominal == 50) || (t10m_nominal == 60));

      if (!a2h_input_eligible) {
        a2h_ineligible_count += 1;
        if (a2h_intervals_accumulated != 0) {
          a2h_reset_count += 1;
        }
        a2h_intervals_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
        aggregation_interval_pending.bit(4) = 0;
#endif
      } else {
        bool a2h_seed = (a2h_intervals_accumulated == 0);
        if (!a2h_seed) {
          if (t10m_generation != a2h_generation ||
              t10m_nominal != a2h_nominal ||
              t10m_sample_rate != a2h_sample_rate ||
              t10m_dc_remove != a2h_dc_remove) {
            a2h_reset_count += 1;
            a2h_seed = true;
          } else if (t10m_out_sequence != a2h_expected_next_seq ||
                     t10m_first_sample != a2h_expected_next_first) {
            a2h_continuity_count += 1;
            a2h_reset_count += 1;
            a2h_seed = true;
          }
        }

        // t10m_out_sequence was incremented above for the record being
        // folded. Use that same value for the cascade's sequence provenance
        // and continuity contract.
        const ap_uint<32> folded_t10m_sequence = t10m_out_sequence;
        if (a2h_seed) {
          a2h_generation = t10m_generation;
          a2h_nominal = t10m_nominal;
          a2h_sample_rate = t10m_sample_rate;
          a2h_dc_remove = t10m_dc_remove;
          a2h_first_sample = t10m_first_sample;
          a2h_first_seq = folded_t10m_sequence;
          a2h_total_samples = t10m_total_samples;
          a2h_total_cycles = t10m_total_cycles;
          a2h_mask_and = t10m_mask_and;
          a2h_arithmetic_flag = t10m_arithmetic_flag;
          a2h_phasor_invalid_or = t10m_phasor_invalid_or;
          a2h_intervals_accumulated = 1;
        } else {
          a2h_total_samples += t10m_total_samples;
          a2h_total_cycles += t10m_total_cycles;
          a2h_mask_and &= t10m_mask_and;
          a2h_arithmetic_flag |= t10m_arithmetic_flag;
          a2h_phasor_invalid_or |= t10m_phasor_invalid_or;
          a2h_intervals_accumulated += 1;
        }
        a2h_last_sample = t10m_last_sample;
        a2h_last_seq = folded_t10m_sequence;
        a2h_expected_next_seq = folded_t10m_sequence + 1;
        a2h_expected_next_first = t10m_first_sample + t10m_total_samples;
        a2h_last_target = t10m_closed_target;
        a2h_last_overshoot = t10m_overshoot_samples;

      a2h_merge_lanes:
        for (int lane = 0; lane < MET_ACTIVE_CHANNELS; ++lane) {
        #pragma HLS PIPELINE off
          const ap_int<128> sum_base =
              a2h_seed ? ap_int<128>(0) : a2h_acc_sum[lane];
          a2h_acc_sum[lane] = sum_base + t10m_acc_sum[lane];
          const ap_uint<128> square_base =
              a2h_seed ? ap_uint<128>(0) : a2h_acc_square[lane];
          a2h_acc_square[lane] = met_add_square_saturating<128>(
              square_base, t10m_acc_square[lane], a2h_arithmetic_flag);
          const ap_int<64> raw_sum_base =
              a2h_seed ? ap_int<64>(0) : a2h_acc_raw_sum[lane];
          a2h_acc_raw_sum[lane] = raw_sum_base + t10m_acc_raw_sum[lane];
          const ap_uint<96> raw_square_base =
              a2h_seed ? ap_uint<96>(0) : a2h_acc_raw_square[lane];
          a2h_acc_raw_square[lane] =
              raw_square_base + t10m_acc_raw_square[lane];
          if (a2h_seed || t10m_acc_minimum[lane] < a2h_acc_minimum[lane]) {
            a2h_acc_minimum[lane] = t10m_acc_minimum[lane];
          }
          if (a2h_seed || t10m_acc_maximum[lane] > a2h_acc_maximum[lane]) {
            a2h_acc_maximum[lane] = t10m_acc_maximum[lane];
          }
          const ap_int<128> re_base =
              a2h_seed ? ap_int<128>(0) : a2h_acc_phasor_re[lane];
          a2h_acc_phasor_re[lane] = met_add_signed_saturating<128>(
              re_base, t10m_acc_phasor_re[lane], a2h_arithmetic_flag);
          const ap_int<128> im_base =
              a2h_seed ? ap_int<128>(0) : a2h_acc_phasor_im[lane];
          a2h_acc_phasor_im[lane] = met_add_signed_saturating<128>(
              im_base, t10m_acc_phasor_im[lane], a2h_arithmetic_flag);
        }
      a2h_merge_power:
        for (int phase = 0; phase < MET_POWER_PHASES; ++phase) {
        #pragma HLS PIPELINE off
          const ap_int<128> power_base =
              a2h_seed ? ap_int<128>(0) : a2h_acc_power[phase];
          a2h_acc_power[phase] = met_add_signed_saturating<128>(
              power_base, t10m_acc_power[phase], a2h_arithmetic_flag);
        }
      a2h_merge_pairs:
        for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
        #pragma HLS PIPELINE off
          const ap_uint<128> vll_base =
              a2h_seed ? ap_uint<128>(0) : a2h_acc_vll_square[pair];
          a2h_acc_vll_square[pair] = met_add_square_saturating<128>(
              vll_base, t10m_acc_vll_square[pair], a2h_arithmetic_flag);
        }

        if (a2h_intervals_accumulated == A2H_INTERVALS_TARGET) {
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
          aggregation_interval_pending.bit(4) = 0;
#endif
          aggregation_interval_pending.bit(2) = 1;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
        } else if (a2h_intervals_accumulated != 0) {
          // A completed ten-minute interval advances the non-normative
          // two-hour preview.  Its cadence is therefore only once per ten
          // minutes and reuses the shared finalizer on a later invocation.
          aggregation_interval_pending.bit(4) = 1;
#endif
        }
      }

      clear_record(record_image);
      fill_envelope(record_image, t10m_out_sequence, t10m_generation,
                    t10m_sample_rate, t10m_count_now, t10m_result_mask,
                    t10m_status, t10m_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = t10m_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = t10m_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = t10m_last_seq;
    t10m_record_lanes:
      for (int lane = 0; lane < MET_CHANNEL_LANES; ++lane) {
      #pragma HLS PIPELINE off
        if (lane < MET_ACTIVE_CHANNELS) {
          const ap_uint<64> rms_units = met_fin_rms_q16(fin_out, lane) >> 16;
          const int base =
              MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS;
          record_image.word[base + 0] = rms_units.range(31, 0);
          record_image.word[base + 1] = rms_units.range(63, 32);
        }
      }
      // MTR2_FREQUENCY_WORD remains zero by construction.
      record_image.word[MTR2_RESET_COUNT_WORD] = t10m_reset_count;
      record_image.word[MTR2_INELIGIBLE_COUNT_WORD] = t10m_ineligible_count;
      record_image.word[MTR2_CONTINUITY_COUNT_WORD] = t10m_continuity_count;
      record_image.word[AGG_LAST_SAMPLE_LOW_WORD] =
          t10m_last_sample.range(31, 0);
      record_image.word[AGG_LAST_SAMPLE_HIGH_WORD] =
          t10m_last_sample.range(63, 32);
    t10m_record_pairs:
      for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      #pragma HLS PIPELINE off
        record_image.word[AGG_VLL_BASE_WORD + pair] =
            ap_uint<64>(met_fin_vll_rms(fin_out, pair) >> 16).range(31, 0);
      }
      record_image.word[TEN_MINUTE_TOTAL_CYCLES_WORD] = t10m_total_cycles;
      record_image.word[TEN_MINUTE_TARGET_SAMPLE_LOW_WORD] =
          t10m_closed_target.range(31, 0);
      record_image.word[TEN_MINUTE_TARGET_SAMPLE_HIGH_WORD] =
          t10m_closed_target.range(63, 32);
      record_image.word[TEN_MINUTE_OVERSHOOT_SAMPLES_WORD] =
          t10m_overshoot_samples;
      serialize_record_format(record_image, MREC_FORMAT_TEN_MINUTE_V1,
                              m_agg);

      // Sibling records reuse the same interval identity.  Detailed UTC
      // boundary provenance is carried only by the fundamental record: the
      // established POWER/PHASOR/UNBAL payload maps own those word numbers.
      // Consumers correlate the siblings through sequence and shape fields.
      // Their status keeps bit 1 available for the existing phasor-invalid
      // contract; completeness remains normative in the fundamental record.
      const ap_uint<32> t10m_sibling_status =
          (ap_uint<32>(t10m_arithmetic_flag)
           << MREC_STATUS_ARITHMETIC_BIT) |
          (ap_uint<32>(t10m_aligned)
           << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) |
          (ap_uint<32>(t10m_contaminated)
           << TEN_MINUTE_STATUS_CONTAMINATED_BIT) |
          (ap_uint<32>(t10m_target_valid)
           << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT);
      const ap_uint<32> t10m_phasor_status =
          t10m_sibling_status |
          (ap_uint<32>(t10m_phasor_invalid_or)
           << PHASOR_STATUS_INVALID_BIT);

      clear_record(record_image);
      fill_envelope(record_image, t10m_out_sequence, t10m_generation,
                    t10m_sample_rate, t10m_count_now, t10m_result_mask,
                    t10m_sibling_status, t10m_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = t10m_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = t10m_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = t10m_last_seq;
      fill_power_payload(fin_out, record_image);
      serialize_record_format(record_image, MREC_FORMAT_TEN_MINUTE_POWER_V1,
                              m_agg);

      clear_record(record_image);
      fill_envelope(record_image, t10m_out_sequence, t10m_generation,
                    t10m_sample_rate, t10m_count_now, t10m_result_mask,
                    t10m_phasor_status, t10m_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = t10m_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = t10m_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = t10m_last_seq;
      fill_phasor_payload(fin_out, record_image);
      serialize_record_format(record_image, MREC_FORMAT_TEN_MINUTE_PHASOR_V2,
                              m_agg);

      clear_record(record_image);
      fill_envelope(record_image, t10m_out_sequence, t10m_generation,
                    t10m_sample_rate, t10m_count_now, t10m_result_mask,
                    t10m_phasor_status, t10m_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = t10m_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = t10m_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = t10m_last_seq;
      fill_unbal_payload(fin_out, record_image);
      serialize_record_format(record_image, MREC_FORMAT_TEN_MINUTE_UNBAL_V2,
                              m_agg);

      t10m_blocks_accumulated = 0;
      // The interval which started at the user-provided target is partial;
      // after its first close the autonomous 600-second target cadence is
      // complete and aligned until software supplies another update.
      t10m_contaminated = 0;
    } else if (pass == 3) {
      // ---- TWO-HOUR-v1: twelve complete/aligned ten-minute intervals. ---
      // Its fundamental payload uses the long-period record layout, with
      // the shape count denoting ten-minute input intervals. Frequency is
      // absent because standardized 10-second frequency remains a direct
      // single-cycle measurement and must not be cascaded.
      const ap_uint<8> a2h_result_mask =
          a2h_mask_and & ap_uint<8>(0x7F);
      const ap_uint<32> a2h_count_now = a2h_total_samples;
      ap_uint<8> a2h_flags = 0;
      a2h_flags.bit(TEN_MINUTE_FLAG_ALIGNED_BIT) = 1;

      a2h_out_sequence += 1;
      const ap_uint<32> a2h_shape_word =
          (ap_uint<32>(A2H_INTERVALS_TARGET)
           << TEN_MINUTE_SHAPE_BLOCKS_LSB) |
          (ap_uint<32>(a2h_nominal) << TEN_MINUTE_SHAPE_NOMINAL_LSB) |
          (ap_uint<32>(a2h_flags) << TEN_MINUTE_SHAPE_FLAGS_LSB);
      const ap_uint<32> a2h_status =
          (ap_uint<32>(a2h_arithmetic_flag)
           << MREC_STATUS_ARITHMETIC_BIT) |
          (ap_uint<32>(1) << TEN_MINUTE_STATUS_COMPLETE_BIT) |
          (ap_uint<32>(1) << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) |
          (ap_uint<32>(1) << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT);

      clear_record(record_image);
      fill_envelope(record_image, a2h_out_sequence, a2h_generation,
                    a2h_sample_rate, a2h_count_now, a2h_result_mask,
                    a2h_status, a2h_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = a2h_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a2h_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a2h_last_seq;
    a2h_record_lanes:
      for (int lane = 0; lane < MET_CHANNEL_LANES; ++lane) {
      #pragma HLS PIPELINE off
        if (lane < MET_ACTIVE_CHANNELS) {
          const ap_uint<64> rms_units = met_fin_rms_q16(fin_out, lane) >> 16;
          const int base = MTR2_CH_BASE_WORD + lane * MTR2_CH_STRIDE_WORDS;
          record_image.word[base + 0] = rms_units.range(31, 0);
          record_image.word[base + 1] = rms_units.range(63, 32);
        }
      }
      record_image.word[MTR2_RESET_COUNT_WORD] = a2h_reset_count;
      record_image.word[MTR2_INELIGIBLE_COUNT_WORD] = a2h_ineligible_count;
      record_image.word[MTR2_CONTINUITY_COUNT_WORD] = a2h_continuity_count;
      record_image.word[AGG_LAST_SAMPLE_LOW_WORD] =
          a2h_last_sample.range(31, 0);
      record_image.word[AGG_LAST_SAMPLE_HIGH_WORD] =
          a2h_last_sample.range(63, 32);
    a2h_record_pairs:
      for (int pair = 0; pair < MET_VLL_PAIRS; ++pair) {
      #pragma HLS PIPELINE off
        record_image.word[AGG_VLL_BASE_WORD + pair] =
            ap_uint<64>(met_fin_vll_rms(fin_out, pair) >> 16).range(31, 0);
      }
      record_image.word[TEN_MINUTE_TOTAL_CYCLES_WORD] = a2h_total_cycles;
      record_image.word[TEN_MINUTE_TARGET_SAMPLE_LOW_WORD] =
          a2h_last_target.range(31, 0);
      record_image.word[TEN_MINUTE_TARGET_SAMPLE_HIGH_WORD] =
          a2h_last_target.range(63, 32);
      record_image.word[TEN_MINUTE_OVERSHOOT_SAMPLES_WORD] =
          a2h_last_overshoot;
      serialize_record_format(record_image, MREC_FORMAT_TWO_HOUR_V1, m_agg);

      const ap_uint<32> a2h_sibling_status =
          (ap_uint<32>(a2h_arithmetic_flag)
           << MREC_STATUS_ARITHMETIC_BIT) |
          (ap_uint<32>(1) << TEN_MINUTE_STATUS_TIME_ALIGNED_BIT) |
          (ap_uint<32>(1) << TEN_MINUTE_STATUS_BOUNDARY_VALID_BIT);
      const ap_uint<32> a2h_phasor_status =
          a2h_sibling_status |
          (ap_uint<32>(a2h_phasor_invalid_or)
           << PHASOR_STATUS_INVALID_BIT);

      clear_record(record_image);
      fill_envelope(record_image, a2h_out_sequence, a2h_generation,
                    a2h_sample_rate, a2h_count_now, a2h_result_mask,
                    a2h_sibling_status, a2h_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = a2h_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a2h_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a2h_last_seq;
      fill_power_payload(fin_out, record_image);
      serialize_record_format(record_image, MREC_FORMAT_TWO_HOUR_POWER_V1,
                              m_agg);

      clear_record(record_image);
      fill_envelope(record_image, a2h_out_sequence, a2h_generation,
                    a2h_sample_rate, a2h_count_now, a2h_result_mask,
                    a2h_phasor_status, a2h_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = a2h_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a2h_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a2h_last_seq;
      fill_phasor_payload(fin_out, record_image);
      serialize_record_format(record_image, MREC_FORMAT_TWO_HOUR_PHASOR_V2,
                              m_agg);

      clear_record(record_image);
      fill_envelope(record_image, a2h_out_sequence, a2h_generation,
                    a2h_sample_rate, a2h_count_now, a2h_result_mask,
                    a2h_phasor_status, a2h_first_sample);
      record_image.word[MTR2_SHAPE_WORD] = a2h_shape_word;
      record_image.word[MTR2_FIRST_BASIC_SEQ_WORD] = a2h_first_seq;
      record_image.word[MTR2_LAST_BASIC_SEQ_WORD] = a2h_last_seq;
      fill_unbal_payload(fin_out, record_image);
      serialize_record_format(record_image, MREC_FORMAT_TWO_HOUR_UNBAL_V2,
                              m_agg);

      a2h_intervals_accumulated = 0;
#if MNC_AGGREGATION_ENABLE_OPEN_PREVIEWS
    } else if (pass == 4) {
      // Non-normative, live view of the open UTC ten-minute accumulator.
      // A separate sequence space and format family prevent this preview
      // from replacing or being confused with a completed measurement.
      const ap_uint<1> aligned =
          t10m_target_valid & ap_uint<1>(!t10m_contaminated);
      ap_uint<8> flags = 0;
      flags.bit(TEN_MINUTE_FLAG_CONTAMINATED_BIT) = t10m_contaminated;
      flags.bit(TEN_MINUTE_FLAG_ALIGNED_BIT) = aligned;
      const ap_uint<32> shape =
          (ap_uint<32>(t10m_blocks_accumulated)
           << TEN_MINUTE_SHAPE_BLOCKS_LSB) |
          (ap_uint<32>(t10m_nominal) << TEN_MINUTE_SHAPE_NOMINAL_LSB) |
          (ap_uint<32>(flags) << TEN_MINUTE_SHAPE_FLAGS_LSB);
      t10m_open_sequence += 1;
      emit_open_interval_records(
          fin_out, record_image, MREC_FORMAT_OPEN_TEN_MINUTE_V1,
          MREC_FORMAT_OPEN_TEN_MINUTE_POWER_V1,
          MREC_FORMAT_OPEN_TEN_MINUTE_PHASOR_V2,
          MREC_FORMAT_OPEN_TEN_MINUTE_UNBAL_V2, t10m_open_sequence,
          t10m_generation,
          t10m_sample_rate, t10m_total_samples,
          t10m_mask_and & ap_uint<8>(0x7F), shape, t10m_first_sample,
          t10m_last_sample, t10m_first_seq, t10m_last_seq,
          t10m_total_cycles, t10m_target_sample, t10m_reset_count,
          t10m_ineligible_count, t10m_continuity_count,
          t10m_arithmetic_flag, t10m_phasor_invalid_or, aligned,
          t10m_contaminated, t10m_target_valid, m_agg);
    } else if (pass == 5) {
      // Non-normative, live view of the open two-hour accumulator.  The
      // expected end is the target of the twelfth ten-minute interval; the
      // completed 2 h record remains the only normative result.
      const ap_uint<64> remaining_intervals =
          ap_uint<64>(A2H_INTERVALS_TARGET - a2h_intervals_accumulated);
      const ap_uint<64> expected_end =
          a2h_last_target + remaining_intervals *
                                ap_uint<64>(a2h_sample_rate) * 600u;
      ap_uint<8> flags = 0;
      flags.bit(TEN_MINUTE_FLAG_ALIGNED_BIT) = 1;
      const ap_uint<32> shape =
          (ap_uint<32>(a2h_intervals_accumulated)
           << TEN_MINUTE_SHAPE_BLOCKS_LSB) |
          (ap_uint<32>(a2h_nominal) << TEN_MINUTE_SHAPE_NOMINAL_LSB) |
          (ap_uint<32>(flags) << TEN_MINUTE_SHAPE_FLAGS_LSB);
      a2h_open_sequence += 1;
      emit_open_interval_records(
          fin_out, record_image, MREC_FORMAT_OPEN_TWO_HOUR_V1,
          MREC_FORMAT_OPEN_TWO_HOUR_POWER_V1,
          MREC_FORMAT_OPEN_TWO_HOUR_PHASOR_V2,
          MREC_FORMAT_OPEN_TWO_HOUR_UNBAL_V2, a2h_open_sequence,
          a2h_generation,
          a2h_sample_rate, a2h_total_samples,
          a2h_mask_and & ap_uint<8>(0x7F), shape, a2h_first_sample,
          a2h_last_sample, a2h_first_seq, a2h_last_seq,
          a2h_total_cycles, expected_end, a2h_reset_count,
          a2h_ineligible_count, a2h_continuity_count,
          a2h_arithmetic_flag, a2h_phasor_invalid_or, ap_uint<1>(1),
          ap_uint<1>(0), ap_uint<1>(1), m_agg);
#endif
    }
  }
}

#undef cycles_in_block
#undef block_cycles_target
#undef block_nominal
#undef block_sample_count
#undef block_first_sample
#undef block_mask
#undef block_locked_and
#undef block_fallback_or
#undef block_phasor_invalid
#undef block_utc_resynchronized
#undef a3s_blocks_accumulated
#undef a3s_agg_generation
#undef a3s_agg_nominal
#undef a3s_agg_sample_rate
#undef a3s_agg_dc_remove
#undef a3s_agg_first_sample
#undef a3s_agg_last_sample
#undef a3s_agg_first_seq
#undef a3s_agg_last_seq
#undef a3s_agg_total_samples
#undef a3s_agg_total_cycles
#undef a3s_mask_and
#undef a3s_freq_sum
#undef a3s_freq_all_valid
#undef a3s_arithmetic_flag
#undef a3s_phasor_invalid_or
#undef a3s_expected_next_seq
#undef a3s_expected_next_first
#undef a3s_utc_overlap
#undef a3s_utc_resynchronized
#undef BASIC_TIER
#undef A3S_TIER
#undef acc_sum
#undef acc_square
#undef acc_raw_sum
#undef acc_raw_square
#undef acc_vll_square
#undef acc_power
#undef acc_minimum
#undef acc_maximum
#undef acc_phasor_re
#undef acc_phasor_im
#undef a3s_acc_sum
#undef a3s_acc_square
#undef a3s_acc_raw_sum
#undef a3s_acc_raw_square
#undef a3s_acc_vll_square
#undef a3s_acc_power
#undef a3s_acc_minimum
#undef a3s_acc_maximum
#undef a3s_acc_phasor_re
#undef a3s_acc_phasor_im
#undef t10m_acc_sum
#undef t10m_acc_square
#undef t10m_acc_raw_sum
#undef t10m_acc_raw_square
#undef t10m_acc_vll_square
#undef t10m_acc_power
#undef t10m_acc_minimum
#undef t10m_acc_maximum
#undef t10m_acc_phasor_re
#undef t10m_acc_phasor_im
#undef a2h_acc_sum
#undef a2h_acc_square
#undef a2h_acc_raw_sum
#undef a2h_acc_raw_square
#undef a2h_acc_vll_square
#undef a2h_acc_power
#undef a2h_acc_minimum
#undef a2h_acc_maximum
#undef a2h_acc_phasor_re
#undef a2h_acc_phasor_im
