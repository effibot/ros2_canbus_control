#include "ros2_waveshare/fixed_frame.hpp"
#include "ros2_waveshare/common.hpp"
#include <numeric>

namespace USBCANBridge {

class FixedSizeFrame : public BaseFrame<FixedSizeFrame> {
// * Implementation of FixedSizeFrame methods called by BaseFrame
protected:
uint8_t calculateChecksum() const {
	/**
	 * @brief Compute checksum over mutable protocol region.
	 * @details Sums bytes: type, frame_type, frame_fmt, id[4], dlc,
	 *          data[8], reserved. Returns low 8 bits.
	 * @return 8-bit checksum value.
	 * @complexity O(1) (fixed number of additions).
	 */

	auto frame_start = frame.begin();
	uint32_t sum = frame[to_uint(FixedSizeIndex::TYPE)] +
	               frame[to_uint(FixedSizeIndex::FRAME_TYPE)] +
	               frame[to_uint(FixedSizeIndex::FRAME_FMT)] +
	               std::accumulate(frame_start + to_uint(FixedSizeIndex::ID_0),
	                               frame_start + to_uint(FixedSizeIndex::ID_3)+1, 0) +
	               frame[to_uint(FixedSizeIndex::DLC)] +
	               std::accumulate(frame_start + to_uint(FixedSizeIndex::DATA_0),
	                               frame_start + to_uint(FixedSizeIndex::DATA_7) + 1, 0) +
	               frame[to_uint(FixedSizeIndex::RESERVED)];
	return static_cast<uint8_t>(sum & 0xFF);
}
};
}; // namespace USBCANBridge