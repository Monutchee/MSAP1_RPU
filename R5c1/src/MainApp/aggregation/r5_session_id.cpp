#include "r5_session_id.hpp"

#include "xil_io.h"
#include "xparameters.h"
#include "xiltimer.h"

#include <cstdint>

namespace msap1::aggregation {

namespace {

constexpr std::uintptr_t system_counter_low_offset = 0x08U;
constexpr std::uintptr_t system_counter_high_offset = 0x0cU;

std::uint64_t read_shared_system_counter() noexcept
{
	/* IOU_SCNTRS is the SoC-wide architectural counter used by the A53s. It
	 * runs independently of an R5C1 remoteproc reset. Read high/low/high so a
	 * low-word rollover cannot synthesize a value that never existed. */
	std::uint32_t high_before{};
	std::uint32_t high_after{};
	std::uint32_t low{};
	do {
		high_before = Xil_In32(XPAR_PSU_IOU_SCNTRS_BASEADDR +
			system_counter_high_offset);
		low = Xil_In32(XPAR_PSU_IOU_SCNTRS_BASEADDR +
			system_counter_low_offset);
		high_after = Xil_In32(XPAR_PSU_IOU_SCNTRS_BASEADDR +
			system_counter_high_offset);
	} while (high_before != high_after);
	return (static_cast<std::uint64_t>(high_after) << 32U) | low;
}

} // namespace

std::uint64_t generate_r5_session_id() noexcept
{
	XTime local_before{};
	XTime local_after{};
	XTime_GetTime(&local_before);
	const auto shared_system_counter = read_shared_system_counter();
	XTime_GetTime(&local_after);
	return derive_r5_session_id({
		.shared_system_counter = shared_system_counter,
		.local_cycle_counter_before =
			static_cast<std::uint64_t>(local_before),
		.local_cycle_counter_after =
			static_cast<std::uint64_t>(local_after),
	});
}

} // namespace msap1::aggregation
