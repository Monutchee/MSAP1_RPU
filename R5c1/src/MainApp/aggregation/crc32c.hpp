#ifndef MSAP1_R5C1_AGGREGATION_CRC32C_HPP
#define MSAP1_R5C1_AGGREGATION_CRC32C_HPP

#include <cstddef>
#include <cstdint>

namespace msap1::aggregation {

/** Calculate reflected CRC-32C over an arbitrary byte sequence. */
[[nodiscard]] std::uint32_t crc32c_bytes(const std::uint8_t *bytes,
	std::size_t byte_count) noexcept;

/**
 * Calculate reflected CRC-32C over little-endian 32-bit words.
 *
 * Polynomial: 0x82F63B78, initial/final XOR: 0xFFFFFFFF.
 */
[[nodiscard]] std::uint32_t crc32c_words(const std::uint32_t *words,
	std::size_t word_count) noexcept;

} // namespace msap1::aggregation

#endif // MSAP1_R5C1_AGGREGATION_CRC32C_HPP
