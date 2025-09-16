#include "ros2_waveshare/fixed_frame.hpp"
#include "ros2_waveshare/common.hpp"
#include <numeric>

namespace USBCANBridge {



// * Implementation of FixedSizeFrame methods called by BaseFrame
/**
 * @brief Compute checksum over mutable protocol region.
 * @details Sums bytes: type, frame_type, frame_fmt, id[4], dlc,
 *          data[8], reserved. Returns low 8 bits.
 * Using the cached frame array for efficiency we can just sum over its elements.
 * @return 8-bit checksum value.
 * @complexity O(1) (fixed number of additions).
 */
uint8_t FixedFrame::calculateChecksum() const {


	uint32_t sum = std::accumulate(frame.begin() + to_uint(FixedSizeIndex::TYPE),
	                               frame.begin() + to_uint(FixedSizeIndex::RESERVED) + 1,
	                               0);
	// Mask to low 8 bits
	return static_cast<uint8_t>(sum & 0xFF);
}

// * Index access operators
uint8_t& FixedFrame::impl_subscript(std::size_t index) {

}

}; // namespace USBCANBridge