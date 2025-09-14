#pragma once
#include "base_frame.hpp"
#include "common.hpp"
#include <array>
#include <cstdint>
#include <cstddef>
namespace USBCANBridge {
// * Variable size frame traits
template<>
struct FrameTraits<VariableSizeFrame> {
	static constexpr std::size_t MAX_FRAME_SIZE = 13; // 1+1+4+8-1
	static constexpr std::size_t MAX_ID_SIZE = 4;
	static constexpr std::size_t MAX_DATA_SIZE = 8;

	using StorageType = std::array<uint8_t, MAX_FRAME_SIZE>;
	using IDType = std::array<uint8_t, MAX_ID_SIZE>;
	using DataType = std::array<uint8_t, MAX_DATA_SIZE>;
	using PayloadPair = std::pair<DataType, uint8_t>;
	using IDPair = std::pair<IDType, uint8_t>;
};

// * Variable size frame class (up to 13 bytes total)
class VariableSizeFrame : public BaseFrame<VariableSizeFrame> {
private:
// Storage for up to 13 bytes
alignas(4) storage frame{};
std::size_t actual_size_ = 0;     // Tracks the actual used size of the frame
static constexpr uint8_t end_byte_ = to_uint8(Constants::END_BYTE);

protected:
};
};