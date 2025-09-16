#pragma once

#include "base_frame.hpp"
#include "common.hpp"
#include <array>
#include <cstdint>
#include <cstddef>

namespace USBCANBridge {
// * Fixed size frame traits
template<>
struct FrameTraits<FixedFrame> {
	static constexpr std::size_t FRAME_SIZE = 20;
	static constexpr std::size_t ID_SIZE = 4;
	static constexpr std::size_t DATA_SIZE = 8;


	using StorageType = std::array<uint8_t, FRAME_SIZE>;
	using IDType = std::array<uint8_t, ID_SIZE>;
	using DataType = std::array<uint8_t, DATA_SIZE>;
	using PayloadPair = std::pair<DataType, uint8_t>;
	using IDPair = std::pair<IDType, uint8_t>;
};
// * Fixed size frame class (20 bytes total)
class FixedFrame : public BaseFrame<FixedFrame> {
private:
// Storage for 20 bytes
alignas(4) storage frame{};
uint8_t checksum_ = 0;  // Cached checksum value
protected:
// Dirty bit to track modifications and trigger checksum recalculation
bool dirty_ = false;
// Checksum calculation helper
uint8_t calculateChecksum() const;

public:
// Constructors
FixedFrame() : BaseFrame() {
	// Initialize fixed frame structure
	frame[to_uint(FixedSizeIndex::HEADER)] = to_uint8(Constants::MSG_HEADER);
	frame[to_uint(FixedSizeIndex::TYPE)] = to_uint8(Type::DATA_FIXED);
	frame[to_uint(FixedSizeIndex::FRAME_TYPE)] = to_uint8(FrameType::STD_FIXED);
	frame[to_uint(FixedSizeIndex::FRAME_FMT)] = to_uint8(FrameFmt::DATA_FIXED);
	frame[to_uint(FixedSizeIndex::RESERVED)] = to_uint8(Constants::RESERVED0);
};

//? declaration of implementation methods called by BaseFrame
// * Index access operators
uint8_t& impl_subscript(std::size_t index);
const uint8_t& impl_subscript(std::size_t index) const;
uint8_t& impl_at(std::size_t index);
const uint8_t& impl_at(std::size_t index) const;
uint8_t* impl_begin();
uint8_t* impl_end();
const uint8_t* impl_begin() const;
const uint8_t* impl_end() const;
std::size_t impl_size() const;
// * Interface for data manipulation
Result<payload> impl_getData() const;
Result<payload> impl_getData(size_t index) const;
Result<bool> impl_setData(const payload& new_data);
// * Interface for type manipulation
Result<Type> impl_getType() const;
Result<bool> impl_setType(Type type);
// * Interface for frame type manipulation
Result<FrameType> impl_getFrameType() const;
Result<bool> impl_setFrameType(FrameType frame_type);
// * Interface for frame format manipulation
Result<FrameFmt> impl_getFrameFmt() const;
Result<bool> impl_setFrameFmt(FrameFmt frame_fmt);
// * Interface for DLC manipulation
Result<uint8_t> impl_getDLC() const;
Result<bool> impl_setDLC(uint8_t dlc);
//* Interface for ID manipulation
Result<id_t> impl_getID() const;
Result<bool> impl_setID(const id_t& id);
// * Interface for serialization/deserialization
Result<storage> impl_serialize() const;
Result<bool> impl_deserialize(const std::vector<uint8_t>& data);
// * Interface for frame validation
Result<bool> impl_validateFrame() const;
Result<bool> impl_validateData() const;
Result<bool> impl_validateID() const;
Result<bool> impl_validateType(Type type) const;
Result<bool> impl_validateFrameType(FrameType frame_type) const;
Result<bool> impl_validateFrameFmt(FrameFmt frame_fmt) const;
Result<bool> impl_validateDLC(uint8_t dlc) const;
// * Additional Checksum validation
Result<bool> validateChecksum() const {
	if (checksum_ == calculateChecksum()) {
		return Result<bool>::success(true);
	} else {
		return Result<bool>::error(Status::WBAD_CHECKSUM);
	}
};

};
}; // namespace USBCANBridge