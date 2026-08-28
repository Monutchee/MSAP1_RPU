#ifndef MSAP1_R5C1_SESSION_ID_HPP
#define MSAP1_R5C1_SESSION_ID_HPP

#include <cstdint>

namespace msap1::aggregation {

[[nodiscard]] constexpr std::uint64_t mix_session_entropy(
	std::uint64_t value) noexcept
{
	value += 0x9e3779b97f4a7c15ULL;
	value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
	value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
	value ^= value >> 31U;
	return value == 0U ? 0x6d736170314d3137ULL : value;
}

/** Non-security boot nonce used only to distinguish volatile R5C1 sessions. */
[[nodiscard]] std::uint64_t generate_r5_session_id() noexcept;

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_SESSION_ID_HPP
