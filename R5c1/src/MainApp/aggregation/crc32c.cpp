#include "crc32c.hpp"

#include <array>

namespace msap1::aggregation {
namespace {

constexpr std::uint32_t polynomial = 0x82F63B78U;

constexpr std::uint32_t table_entry(std::uint32_t value) noexcept
{
	for (unsigned bit = 0U; bit < 8U; ++bit)
		value = (value >> 1U) ^ ((value & 1U) ? polynomial : 0U);
	return value;
}

constexpr std::array<std::uint32_t, 256U> make_table() noexcept
{
	std::array<std::uint32_t, 256U> table{};
	for (std::size_t index = 0U; index < table.size(); ++index)
		table[index] = table_entry(static_cast<std::uint32_t>(index));
	return table;
}

constexpr auto table = make_table();

std::uint32_t update_byte(std::uint32_t crc, std::uint8_t byte) noexcept
{
	return table[(crc ^ byte) & 0xffU] ^ (crc >> 8U);
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
