#include "aggregation_frame_ring.hpp"

namespace msap1::aggregation {

bool AggregationFrameRing::try_push(const AggregationFrame &frame) noexcept
{
	const auto write = __atomic_load_n(&write_index_, __ATOMIC_RELAXED);
	const auto read = __atomic_load_n(&read_index_, __ATOMIC_ACQUIRE);
	if (write - read >= capacity)
		return false;

	frames_[write % capacity] = frame;
	__atomic_store_n(&write_index_, write + 1U, __ATOMIC_RELEASE);
	return true;
}

bool AggregationFrameRing::try_pop(AggregationFrame &frame) noexcept
{
	const auto read = __atomic_load_n(&read_index_, __ATOMIC_RELAXED);
	const auto write = __atomic_load_n(&write_index_, __ATOMIC_ACQUIRE);
	if (read == write)
		return false;

	frame = frames_[read % capacity];
	__atomic_store_n(&read_index_, read + 1U, __ATOMIC_RELEASE);
	return true;
}

std::size_t AggregationFrameRing::size() const noexcept
{
	const auto write = __atomic_load_n(&write_index_, __ATOMIC_ACQUIRE);
	const auto read = __atomic_load_n(&read_index_, __ATOMIC_ACQUIRE);
	return static_cast<std::size_t>(write - read);
}

} // namespace msap1::aggregation
