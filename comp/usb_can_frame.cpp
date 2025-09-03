#include "usb_can_frame.hpp"

namespace USBCANBridge  // Changed to match header namespace
{
class USBCANFrame {
protected:
USBCANAdapterBaseFrame* base_frame_ptr;
public:
USBCANFrame() : base_frame_ptr(nullptr) {
}
virtual ~USBCANFrame() = default;
virtual std::vector<uint8_t> serialize() const = 0;
virtual bool deserialize(const std::vector<uint8_t>& data) = 0;
virtual USBCANFrameType getType() const = 0;
virtual size_t getFrameSize() const = 0;
virtual bool isValid() const = 0;
bool hasValidStart() const {
	return base_frame_ptr && base_frame_ptr->isValidStart();
}
};

class USBCANFrame20Byte : public USBCANFrame {
protected:
USBCANAdapter20ByteFrame frame_data;
public:
USBCANFrame20Byte() : USBCANFrame() {
	base_frame_ptr = &frame_data;
}
uint8_t calculateChecksum(const USBCANAdapter20ByteFrame& frame) const {
	/**
	 * @brief Calculate checksum for a 20-byte USB-CAN-A adapter frame
	 * The checksum is the low 8 bits of the cumulative sum from the frame type byte
	 * to the reserved byte (inclusive).
	 * @param frame The 20-byte frame for which to calculate the checksum
	 * @return The calculated checksum byte
	 */

	// Init
	uint32_t sum = 0;
	// Sum bytes from frame type (data[0]) to reserved (data[17])
	sum += frame.data[0];         // Type byte
	sum += std::accumulate(frame.data.begin() + 1, frame.data.end(), 0U);
	// Return low 8 bits
	return static_cast<uint8_t>(sum & 0xFF);

}
bool validateChecksum(const USBCANAdapter20ByteFrame& frame) const {
	/**
	 * @brief Validate checksum for a 20-byte USB-CAN-A adapter frame
	 * Compares the stored checksum byte with a newly calculated checksum.
	 * @param frame The 20-byte frame to validate
	 * @return True if the checksum is valid, false otherwise
	 */
	return frame.checksum == calculateChecksum(frame);
}
std::vector<uint8_t> serialize() const override {
	/**
	 * @brief Serialize the 20-byte USB-CAN-A adapter frame to a byte vector
	 * This method converts the frame structure into a byte vector suitable for
	 * transmission or storage. It includes the start byte, message header, data bytes,
	 * and checksum.
	 * @return A vector of bytes representing the serialized frame
	 */

	//* Checksum is calculated during serialization to ensure it reflects current data
	const_cast<USBCANFrame20Byte*>(this)->frame_data.checksum = calculateChecksum(this->frame_data);
	// Allocate vector and fill it
	std::vector<uint8_t> serialized;
	serialized.reserve(20);
	// Fill vector with frame data
	serialized.push_back(frame_data.start_byte);
	serialized.push_back(frame_data.msg_header);
	serialized.insert(serialized.end(), frame_data.data.begin(), frame_data.data.end());
	serialized.push_back(frame_data.checksum);
	return serialized;
}
bool deserialize(const std::vector<uint8_t>& raw_data) override {
	/**
	 * @brief Deserialize a 20-byte USB-CAN-A adapter frame from a byte vector
	 * This method populates the frame structure from the provided byte vector.
	 * It checks for correct size and copies the data into the frame structure.
	 * This method overwrites existing frame data of the calling object.
	 * @param raw_data The byte vector containing the serialized frame data
	 * @return True if deserialization is successful, false otherwise
	 */
	// Check size of the input data
	if (raw_data.size() != 20) return false;

	//? As the deserialization overwrites the existing data, the header bytes are already set

	//* Just copy the data bytes from the input vector
	std::copy(raw_data.begin() + 2, raw_data.begin() + 19, frame_data.data.begin());
	// Copy checksum byte
	frame_data.checksum = raw_data[19];
	// Validate checksum
	return validateChecksum(frame_data);
};

USBCANFrameType getType() const override {
	/**
	 * @brief Determine the frame type (standard or extended) based on the frame data
	 * The frame type is determined by the value in data[3]:
	 * - 0x01 indicates a standard frame
	 * - 0x02 indicates an extended frame
	 * @return The frame type as a USBCANFrameType enum value
	 */

	// retrieve the frame type byte
	USBCANFrameType frame_type_byte = static_cast<USBCANFrameType>(frame_data.getFrameType());
	// Determine frame type based on the byte value
	if (frame_type_byte == USBCANFrameType::STD_FIXED) {
		return USBCANFrameType::STD_FIXED;
	} else if (frame_type_byte == USBCANFrameType::EXT_FIXED) {
		return USBCANFrameType::EXT_FIXED;
	} else {
		throw std::runtime_error("Unknown frame type in USBCANFrame20Byte");
	}

}
size_t getFrameSize() const override {
	return 20;
}
bool isValid() const override {
	return hasValidStart() && validateChecksum(frame_data);
}
void setDataBytes(const std::vector<uint8_t>& data) {
	/**
	 * @brief Set the data bytes of the frame
	 * This method allows setting the data bytes of the frame.
	 * It ensures that the data size does not exceed 17 bytes.
	 * @param data The vector of data bytes to set
	 */
	if (data.size() > 17) {
		const std::string msg = "Data size is " + std::to_string(data.size()) + ", but must be 17 bytes or less for USBCANAdapter20ByteFrame";
		throw std::invalid_argument(msg);
	}
	(this)->frame_data.setData(data);
}
uint8_t getDataByte(size_t index) const {
	return frame_data.at(index);
}
void setDataByte(size_t index, uint8_t value) {
	frame_data.at(index) = value;
}
};


} // namespace USBCANBridge
int main() {
	// Example usage
	USBCANBridge::USBCANFrame20Byte frame;
	std::vector<uint8_t> example_data ={
		0x01, // Type byte
		0x01, // Frame type (standard)
		0x01, // Frame format (data frame)
		0x23, 0x01, 0x00, 0x00, // ID (0x00000123)
		0x07, // DLC (8 bytes)
		0x1, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, // Data bytes
		0x00 // Reserved
	};
	frame.setDataBytes(example_data);
	std::vector<uint8_t> data = frame.serialize();
	// print serialized data
	for (auto byte : data) {
		printf("%02X ", byte);
	}
	printf("\n");
	return 0;

}

