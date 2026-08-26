#include "crc32c.hpp"

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t polynomial = 0x82F63B78U;

std::uint32_t update_byte(std::uint32_t crc, std::uint8_t byte) noexcept
{
	crc ^= byte;
	for (unsigned bit = 0; bit < 8U; ++bit)
		crc = (crc >> 1U) ^ ((crc & 1U) ? polynomial : 0U);
	return crc;
}

} // namespace

std::uint32_t crc32c_bytes(const std::uint8_t *bytes,
	std::size_t byte_count) noexcept
{
	std::uint32_t crc = 0xFFFFFFFFU;
	for (std::size_t index = 0; index < byte_count; ++index)
		crc = update_byte(crc, bytes[index]);
	return crc ^ 0xFFFFFFFFU;
}

std::uint32_t crc32c_words(const std::uint32_t *words,
	std::size_t word_count) noexcept
{
	std::uint32_t crc = 0xFFFFFFFFU;
	for (std::size_t index = 0; index < word_count; ++index) {
		const std::uint32_t word = words[index];
		crc = update_byte(crc, static_cast<std::uint8_t>(word));
		crc = update_byte(crc, static_cast<std::uint8_t>(word >> 8U));
		crc = update_byte(crc, static_cast<std::uint8_t>(word >> 16U));
		crc = update_byte(crc, static_cast<std::uint8_t>(word >> 24U));
	}
	return crc ^ 0xFFFFFFFFU;
}

} // namespace msap1::aggregation
