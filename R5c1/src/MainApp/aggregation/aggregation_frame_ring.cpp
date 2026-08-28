#include "aggregation_frame_ring.hpp"

#include <algorithm>

namespace msap1::aggregation {

bool AggregationFrameRing::try_push(const AggregationFrame &frame) noexcept
{
	const auto write = __atomic_load_n(&write_index_, __ATOMIC_RELAXED);
	const auto read = __atomic_load_n(&read_index_, __ATOMIC_ACQUIRE);
	if (write - read >= capacity)
		return false;

	auto &destination = frames_[write % capacity];
	destination.word_count = frame.word_count;
	std::copy_n(frame.words.begin(), frame.word_count,
		destination.words.begin());
	__atomic_store_n(&write_index_, write + 1U, __ATOMIC_RELEASE);
	return true;
}

bool AggregationFrameRing::try_pop(AggregationFrame &frame) noexcept
{
	const auto read = __atomic_load_n(&read_index_, __ATOMIC_RELAXED);
	const auto write = __atomic_load_n(&write_index_, __ATOMIC_ACQUIRE);
	if (read == write)
		return false;

	const auto &source = frames_[read % capacity];
	frame.word_count = source.word_count;
	std::copy_n(source.words.begin(), source.word_count, frame.words.begin());
	__atomic_store_n(&read_index_, read + 1U, __ATOMIC_RELEASE);
	return true;
}

std::size_t AggregationFrameRing::size() const noexcept
{
	const auto write = __atomic_load_n(&write_index_, __ATOMIC_ACQUIRE);
	const auto read = __atomic_load_n(&read_index_, __ATOMIC_ACQUIRE);
	return static_cast<std::size_t>(write - read);
}

std::size_t AggregationFrameRing::available_capacity() const noexcept
{
	const auto used = size();
	return used >= capacity ? 0U : capacity - used;
}

} // namespace msap1::aggregation
