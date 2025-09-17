#include "ros2_waveshare/fixed_frame.hpp"
#include "ros2_waveshare/common.hpp"
#include "ros2_waveshare/error.hpp"
#include <numeric>

namespace USBCANBridge {
// * Implementation of FixedSizeFrame methods called by BaseFrame
/**
 * @brief Fast checksum calculation for fixed size frame protocol.
 *
 * Sums bytes: type, frame_type, frame_fmt, id[4], dlc, data[8], reserved.
 * Returns low 8 bits.
 *
 * @return 8-bit checksum value.
 */
uint8_t FixedFrame::calculateChecksum() const {
	uint32_t sum = std::accumulate(frame.begin() + to_uint(FixedSizeIndex::TYPE),
	                               frame.begin() + to_uint(FixedSizeIndex::RESERVED) + 1,
	                               0);
	// Mask to low 8 bits
	return static_cast<uint8_t>(sum & 0xFF);
};

// * Index access operators
/**
 * @brief Subscript operator for direct byte access without bounds checking.
 * @param index Byte index (0-19).
 * @return Reference to byte at specified index.
 */
uint8_t& FixedFrame::impl_subscript(std::size_t index) {
	return frame[index];
};
/**
 * @brief Const subscript operator for direct byte access without bounds checking.
 * @param index Byte index (0-19).
 * @return Const reference to byte at specified index.
 */
const uint8_t& FixedFrame::impl_subscript(std::size_t index) const {
	return frame[index];
};
/**
 * @brief Non-const access to the byte at the specified index.
 *
 * @param index Byte index (0-19).
 * @return Reference to byte at specified index.
 */
uint8_t& FixedFrame::impl_at(std::size_t index) {
	if (index >= frame.size()) {
		throw std::out_of_range("Index out of range");
	};
	return frame[index];
};
/**
 * @brief Const access to the byte at the specified index.
 *
 * @param index Byte index (0-19).
 * @return Const reference to byte at specified index.
 */
const uint8_t& FixedFrame::impl_at(std::size_t index) const {
	return const_cast<FixedFrame*>(this)->impl_at(index);
};
/**
 * @brief Non-const access to the first byte of the frame data.
 *
 * @return uint8_t*
 */
uint8_t* FixedFrame::impl_begin() {
	return frame.data();
};
/**
 * @brief Non-const access to the last byte of the frame data.
 *
 * @return uint8_t*
 */
uint8_t* FixedFrame::impl_end() {
	return frame.data() + frame.size();
};
/**
 * @brief Const access to the first byte of the frame data.
 *
 * @return const uint8_t*
 */
const uint8_t* FixedFrame::impl_begin() const {
	return frame.data();
};
/**
 * @brief Const access to the last byte of the frame data.
 *
 * @return const uint8_t*
 */
const uint8_t* FixedFrame::impl_end() const {
	return frame.data() + frame.size();
};
/**
 * @brief Get the size of the frame.
 *
 * When called on a FixedFrame, this will always return 20.
 *
 * @return std::size_t
 */
std::size_t FixedFrame::impl_size() const {
	return frame.size();
};
// * Interface for data field manipulation
/**
 * @brief Get the data field from the frame.
 *
 * A fast copy of the 8 data bytes and the DLC is returned.
 *
 * @return Result<FixedFrame::payload> Payload data with DLC on success, error code on failure.
 */
Result<FixedFrame::payload> FixedFrame::impl_getData() const {

	FixedFrame::payload data{};
	data.second = frame[to_uint(FixedSizeIndex::DLC)];
	auto frame_start = frame.begin() + to_uint(FixedSizeIndex::DATA_0);
	auto frame_end = frame_start + 8; //* this includes all 8 data bytes

	std::copy(frame_start, frame_end, data.first.begin());

	return Result<payload>::success(data);
};

/**
 * @brief Get a copy of the data field from the frame at the specified index.
 *
 * If index is out of range (>=8), returns WBAD_DATA_INDEX error.
 *
 * @param index Data byte index (0-7).
 * @return Result<FixedFrame::payload> Payload data with DLC on success, error code on failure.
 */
Result<uint8_t> FixedFrame::impl_getData(size_t index) const {
	if (index >= 8) {
		return Result<uint8_t>::error(Status::WBAD_DATA_INDEX);
	}
	return Result<uint8_t>::success(frame[to_uint(FixedSizeIndex::DATA_0) + index]);
};

Result<bool> FixedFrame::impl_setData(const payload& new_data) {
	if (new_data.second > 8) {
		return Result<bool>::error(Status::WBAD_DLC);
	}
	// Copy new data bytes
	auto frame_start = frame.begin() + to_uint(FixedSizeIndex::DATA_0);
	std::copy(new_data.first.begin(), new_data.first.begin() + new_data.second, frame_start);
	// Zero-fill unused bytes
	std::fill(frame_start + new_data.second, frame_start + 8, 0);
	// Update DLC
	frame[to_uint(FixedSizeIndex::DLC)] = new_data.second;
	return Result<bool>::success(true);
};

/**
 * @brief Set the data field in the frame at the specified index.
 *
 * If index is out of range (>=8), returns WBAD_DATA_INDEX error.
 *
 * @param index Data byte index (0-7).
 * @param value New value for the data byte.
 * @return Result<bool> Success or error code.
 */
Result<bool> FixedFrame::impl_setData(size_t index, uint8_t value) {
	if (index >= 8) {
		return Result<bool>::error(Status::WBAD_DATA_INDEX);
	}
	frame[to_uint(FixedSizeIndex::DATA_0) + index] = value;
	return Result<bool>::success(true);
};

// * Interface for type field manipulation
//* Note: we expect validation to be done in BaseFrame before calling these methods and at set time, so no additional validation is performed here.
/**
 * @brief Get the Type field from the frame.
 * @return Result<Type> Type on success, error code on failure.
 */
Result<Type> FixedFrame::impl_getType() const {
	return Result<Type>::success(static_cast<Type>(frame[to_uint(FixedSizeIndex::TYPE)]));
};

/**
 * @brief Set the Type field in the frame.
 * @param type New Type value.
 * @return Result<bool> Success or error code.
 */
Result<FrameType> FixedFrame::impl_getFrameType() const {
	return Result<FrameType>::success(
		static_cast<FrameType>(frame[to_uint(FixedSizeIndex::FRAME_TYPE)]));
};

/**
 * @brief Set the FrameType field in the frame.
 * @param frame_type New FrameType value.
 * @return Result<bool> Success or error code.
 */

Result<bool> FixedFrame::impl_setFrameType(FrameType frame_type) {
	frame[to_uint(FixedSizeIndex::FRAME_TYPE)] = to_uint8(frame_type);
	return Result<bool>::success(true);
};

/**
 * @brief Get the FrameFmt field from the frame.
 * @return Result<FrameFmt> FrameFmt on success, error code on failure.
 */
Result<FrameFmt> FixedFrame::impl_getFrameFmt() const {
	return Result<FrameFmt>::success(
		static_cast<FrameFmt>(frame[to_uint(FixedSizeIndex::FRAME_FMT)]));
};
/**
 * @brief Set the FrameFmt field in the frame.
 * @param frame_fmt New FrameFmt value.
 * @return Result<bool> Success or error code.
 */
Result<bool> FixedFrame::impl_setFrameFmt(FrameFmt frame_fmt) {
	frame[to_uint(FixedSizeIndex::FRAME_FMT)] = to_uint8(frame_fmt);
	return Result<bool>::success(true);
};
/**
 * @brief Get the DLC field from the frame.
 * @return Result<uint8_t> DLC on success, error code on failure.
 */
Result<uint8_t> FixedFrame::impl_getDLC() const {
	return Result<uint8_t>::success(frame[to_uint(FixedSizeIndex::DLC)]);
};
/**
 * @brief Set the DLC field in the frame.
 * @param dlc New DLC value (0-8).
 * @return Result<bool> Success or error code.
 */
Result<bool> FixedFrame::impl_setDLC(uint8_t dlc) {
	if (dlc > 8) {
		return Result<bool>::error(Status::WBAD_DLC);
	}
	frame[to_uint(FixedSizeIndex::DLC)] = dlc;
	return Result<bool>::success(true);
};
/**
 * @brief Get the ID field from the frame.
 * @return Result<FixedFrame::frmID> ID on success, error code on failure.
 */
Result<FixedFrame::frmID> FixedFrame::impl_getID() const {
	FixedFrame::frmID id{};
	auto id_start = frame.begin() + to_uint(FixedSizeIndex::ID_0);
	auto id_end = id_start + 4; //* this includes all 4 ID bytes
	std::copy(id_start, id_end, id.first.begin());
	return Result<frmID>::success(id);
};
/**
 * @brief Set the ID field in the frame.
 * @param id New ID value (32-bit).
 * @return Result<bool> Success or error code.
 */
Result<bool> FixedFrame::impl_setID(const frmID& id) {
	std::copy(id.first.begin(), id.first.end(), frame.begin() + to_uint(FixedSizeIndex::ID_0));
	return Result<bool>::success(true);
};

// * Interface for serialization/deserialization



// * Interface for frame validation
/**
 * @brief Validate the entire frame structure.
 *
 * Before any of the checks below, we check if the dirty-bit has been set, and if so, we recalculate the checksum.
 *
 * The following checks are performed:
 * - Start byte is correct (0xAA)
 * - Header byte is correct (0x55)
 * - [Skipped] Type field is valid for FixedFrame (DATA_FIXED)
 * - [Skipped] FrameType field is valid (STD_FIXED or EXT_FIXED)
 * - [Skipped] FrameFmt field is valid (DATA_FIXED or REMOTE_FIXED)
 * - [Skipped] ID field is valid (32-bit, with 16-bit restriction for STD_FIXED)
 * - DLC field is in range (0-8) and consistent with data bytes
 * - Reserved byte is correct (0x00)
 * - Checksum matches calculated value
 *
 * Skipped checks are omitted as they are enforced at set time via the respective validate methods. This avoids redundant validation.
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateFrame() const {
	// Recalculate checksum if dirty
	if (dirty_) {
		checksum_ = calculateChecksum();
		dirty_ = false;
	}

	// Validate start and header bytes - just in case they were modified directly
	if (frame[to_uint(FixedSizeIndex::START)] != to_uint8(Constants::START_BYTE)) {
		return Result<bool>::error(Status::WBAD_START);
	}
	if (frame[to_uint(FixedSizeIndex::HEADER)] != to_uint8(Constants::MSG_HEADER)) {
		return Result<bool>::error(Status::WBAD_HEADER);
	}

	// Validate DLC
	auto dlc = frame[to_uint(FixedSizeIndex::DLC)];
	auto dlc_validation = impl_validateDLC(dlc);
	if (dlc_validation.fail()) {
		return Result<bool>::error(Status::WBAD_DLC);
	}
	// Validate reserved byte
	if (frame[to_uint(FixedSizeIndex::RESERVED)] != to_uint8(Constants::RESERVED0)) {
		return Result<bool>::error(Status::WBAD_RESERVED);
	}
	// Validate checksum
	auto checksum_validation = validateChecksum();
	if (checksum_validation.fail()) {
		return Result<bool>::error(Status::WBAD_CHECKSUM);
	}
	return Result<bool>::success(true);
}

// * Validate specific sections during set operations
/**
 * @brief Validate the data field before updating the frame.
 *
 * For a fixed frame, we only need to check that the DLC is in range (0-8).
 *
 * @param data
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateData(const payload& data) const {
	if (data.second > 8) {
		return Result<bool>::error(Status::WBAD_DATA);
	}
	return Result<bool>::success(true);
}

/**
 * @brief Validate the data index before updating the frame.
 *
 * For a fixed frame, we only need to check that the index is in range (0-7).
 *
 * @param index
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateDataIndex(const size_t index) const {
	if (index > 7) {
		return Result<bool>::error(Status::WBAD_DATA_INDEX);
	}
	return Result<bool>::success(true);
}

/**
 * @brief Validate the ID field before updating the frame.
 *
 * For a fixed frame, we accept all 32-bit IDs but if the FrameType is STD_FIXED, we only accept 16-bit IDs.
 *! If you want to set a specific ID, first check the FrameType and adjust accordingly.
 *
 * @param id
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateID(const frmID& id) const {
	auto frame_type_res = impl_getFrameType();
	if (frame_type_res.fail()) {
		return Result<bool>::error(Status::WBAD_FRAME_TYPE);
	}
	auto frame_type = frame_type_res.value;
	// If standard fixed frame, only allow 16-bit IDs (upper 16 bits must be zero)
	auto value = id.first;
	if (frame_type == FrameType::STD_FIXED && (value[2] != 0 || value[3] != 0)) {
		return Result<bool>::error(Status::WBAD_ID);
	}
	return Result<bool>::success(true);
}
/**
 * @brief Validate the Type field before updating the frame.
 *
 * For a fixed frame, we only accept DATA_FIXED type as CONF_FIXED is dedicated to configuration frames implementation.
 *
 * @param type
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateType(const Type& type) const {
	if (type != Type::DATA_FIXED) {
		return Result<bool>::error(Status::WBAD_TYPE);
	}
	return Result<bool>::success(true);
}
/**
 * @brief Validate the FrameType field before updating the frame.
 *
 * For a fixed frame, we only accept STD_FIXED and EXT_FIXED types.
 *
 * @param frame_type
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateFrameType(const FrameType& frame_type) const {
	// Accept only defined FrameType values
	switch (frame_type) {
	case FrameType::STD_FIXED:
	case FrameType::EXT_FIXED:
		return Result<bool>::success(true);
	default:
		return Result<bool>::error(Status::WBAD_FRAME_TYPE);
	}
}
/**
 * @brief Validate the FrameFmt field before updating the frame.
 *
 * For a fixed frame, we only accept DATA_FIXED and REMOTE_FIXED formats.
 *
 * @param frame_fmt
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateFrameFmt(const FrameFmt& frame_fmt) const {
	// Accept only defined FrameFmt values
	switch (frame_fmt) {
	case FrameFmt::DATA_FIXED:
	case FrameFmt::REMOTE_FIXED:
		return Result<bool>::success(true);
	default:
		return Result<bool>::error(Status::WBAD_FORMAT);
	}
}
/**
 * @brief Validate the DLC field before updating the frame.
 *
 * For a fixed frame, we only accept DLC values in range 0-8.
 *
 * @param dlc
 * @return Result<bool>
 */
Result<bool> FixedFrame::impl_validateDLC(const uint8_t& dlc) const {
	if (dlc > 8) {
		return Result<bool>::error(Status::WBAD_DLC);
	}
	// Check that data bytes beyond DLC are zero
	for (size_t i = dlc; i < 8; ++i) {
		if (frame[to_uint(FixedSizeIndex::DATA_0) + i] != 0) {
			return Result<bool>::error(Status::WBAD_DLC);
		}
	}
	return Result<bool>::success(true);
}

};// namespace USBCANBridge