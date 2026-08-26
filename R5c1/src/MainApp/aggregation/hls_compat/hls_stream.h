#ifndef MSAP1_R5C1_HLS_COMPAT_STREAM_H
#define MSAP1_R5C1_HLS_COMPAT_STREAM_H

#include <cstddef>

namespace hls {

/**
 * Small, allocation-free compatibility stream for executing the shared
 * aggregation arithmetic on R5C1.
 *
 * The Vitis HLS implementation of hls::stream uses host-only synchronization
 * and exception machinery.  R5C1 needs only the empty/read/write contract
 * used by AggregationEngine, so a bounded static queue is both sufficient and
 * deterministic.  Capacity is intentionally large enough for one four-record
 * result family (4 x 64 words) without a consumer running concurrently.
 */
template <typename T, std::size_t Capacity = 256U>
class stream final {
public:
	[[nodiscard]] bool empty() const noexcept { return size_ == 0U; }
	[[nodiscard]] bool full() const noexcept { return size_ == Capacity; }
	[[nodiscard]] std::size_t size() const noexcept { return size_; }

	bool write_nb(const T &value) noexcept
	{
		if (full())
			return false;
		storage_[tail_] = value;
		tail_ = (tail_ + 1U) % Capacity;
		++size_;
		return true;
	}

	void write(const T &value) noexcept
	{
		// The aggregation wrapper drains after every engine invocation, so a
		// full queue indicates a programming/contract error.  Retain the first
		// complete family and expose the condition to the wrapper instead of
		// allocating or throwing from real-time firmware.
		if (!write_nb(value))
			overflowed_ = true;
	}

	bool read_nb(T &value) noexcept
	{
		if (empty())
			return false;
		value = storage_[head_];
		head_ = (head_ + 1U) % Capacity;
		--size_;
		return true;
	}

	T read() noexcept
	{
		T value{};
		if (!read_nb(value))
			underflowed_ = true;
		return value;
	}

	[[nodiscard]] bool overflowed() const noexcept { return overflowed_; }
	[[nodiscard]] bool underflowed() const noexcept { return underflowed_; }

	void clear_errors() noexcept
	{
		overflowed_ = false;
		underflowed_ = false;
	}

private:
	T storage_[Capacity]{};
	std::size_t head_{};
	std::size_t tail_{};
	std::size_t size_{};
	bool overflowed_{};
	bool underflowed_{};
};

} // namespace hls

#endif // MSAP1_R5C1_HLS_COMPAT_STREAM_H
