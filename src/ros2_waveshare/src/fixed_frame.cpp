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
Result<FixedFrame::payload> FixedFrame::impl_getData() const {

	FixedFrame::payload data{};
	data.second = frame[to_uint(FixedSizeIndex::DLC)];
	auto frame_start = frame.begin() + to_uint(FixedSizeIndex::DATA_0);
	auto frame_end = frame_start + 8; //* this includes all 8 data bytes

	std::copy(frame_start, frame_end, data.first.begin());

	return Result<payload>::success(data);
};

Result<FixedFrame::payload> FixedFrame::impl_getData(size_t index) const {
	if (index >= 8) {
		return Result<payload>::error(Status::WBAD_DATA_INDEX);
	}
	FixedFrame::payload data{};;
	data.second = frame[to_uint(FixedSizeIndex::DLC)];
	std::copy(frame.begin() + to_uint(FixedSizeIndex::DATA_0),
	          frame.begin() + to_uint(FixedSizeIndex::DATA_0) + 8,
	          data.first.begin());
	return Result<payload>::success(data);

}; // namespace USBCANBridge
}