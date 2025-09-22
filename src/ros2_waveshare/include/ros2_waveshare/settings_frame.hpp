/**
 * @file settings_frame.hpp
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @brief Implementation of the Setting Frame class for Waveshare USB-CAN-A adapter. This frame type is used to configure the adapter settings and should be used only when is needed to change the adapter configuration (i.e., on the startup of the application).
 * The implementation uses the CRTP pattern for static polymorphism and efficiency, exploiting the implementation of the FixedFrame class.
 * @version 0.1
 * @date 2025-09-20
 *
 * @copyright Copyright (c) 2025
 */
#pragma once
#include "fixed_frame.hpp"

namespace USBCANBridge {
// * Settings frame traits
template<>
struct FrameTraits<SettingsFrame> {
	static constexpr std::size_t FRAME_SIZE = 20; // Same as FixedFrame
	using StorageType = std::array<uint8_t, FRAME_SIZE>;
};
// * Settings frame class (20 bytes total, same as FixedFrame)
class SettingsFrame : public FixedFrame {
private:
// No additional members needed, reuse FixedFrame storage and methods
protected:
// No additional protected members needed
public:
// Constructors
SettingsFrame(Type type=Type::CONF_VAR, CANBaud rate=CANBaud::SPEED_1000K, FrameType frame_type=FrameType::STD_FIXED,
              std::array<std::byte, 4> filter_id = {}, std::array<std::byte, 4> mask_id = {}, CANMode mode = CANMode::NORMAL, RTX rtx = RTX::AUTO) : FixedFrame() {
	// Reuse FixedFrame constructor to initialize common fields
	// Override specific fields for SettingsFrame
	frame[to_uint(ConfigCommandIndex::TYPE)] = to_uint8(type);
	frame[to_uint(ConfigCommandIndex::CAN_BAUD)] = to_uint8(rate);
	frame[to_uint(ConfigCommandIndex::FRAME_TYPE)] = to_uint8(frame_type);
	// Set Filter and Mask IDs (reuse the loop for the reserved bytes)
	for (size_t i = 0; i < 4; ++i) {
		frame[to_uint(ConfigCommandIndex::FILTER_ID_1) + i] = static_cast<uint8_t>(filter_id[i]);
		frame[to_uint(ConfigCommandIndex::MASK_ID_1) + i] = static_cast<uint8_t>(mask_id[i]);
		frame[to_uint(ConfigCommandIndex::BACKUP_0) + i] = to_uint8(Constants::RESERVED0);
	}
	// Set CAN Mode and RTX
	frame[to_uint(ConfigCommandIndex::CAN_MODE)] = to_uint8(mode);
	frame[to_uint(ConfigCommandIndex::AUTO_RTX)] = to_uint8(rtx);
	// Initialize checksum
	checksum_ = calculateChecksum();
}


/**
 * The following methods needs to override the FixedFrame implementations to provide
 * correct validation logic specific to SettingsFrame. However, Settings frame does
 * not define ID, DLC or Data fields, so we need modify how these validations are performed.
 * We also need to restrict the allowed values for Type and FrameType fields.
 */

//? declaration of implementation methods called by BaseFrame
// * Index access operators


// * Interface for fields validation that are relevant for SettingsFrame
Result<bool> impl_validateFrame() const;
Result<bool> impl_validateType(const Type& type) const;
Result<bool> impl_validateFrameType(const FrameType& frame_type) const;
// We reuse the header and checksum validation from FixedFrame
Result<bool> validateCanMode() const {
	auto mode = static_cast<CANMode>(frame[to_uint(ConfigCommandIndex::CAN_MODE)]);
	if (mode == CANMode::NORMAL || mode == CANMode::LOOPBACK || mode == CANMode::SILENT || mode == CANMode::LOOPBACK_SILENT) {
		return Result<bool>::success(true);
	}
	return Result<bool>::error(Status::WBAD_CAN_MODE);
}
Result<bool> validateRTX() const {
	auto rtx = static_cast<RTX>(frame[to_uint(ConfigCommandIndex::AUTO_RTX)]);
	if (rtx == RTX::AUTO || rtx == RTX::OFF) {
		return Result<bool>::success(true);
	}
	return Result<bool>::error(Status::WBAD_CAN_MODE);
}

// * The following validations are not applicable for SettingsFrame, so we provide no-op implementations
void impl_validateFrameFmt(const FrameFmt& frame_fmt) const {
	return;
}
void impl_validateID(const frmID& id) const {
	return;
}
void impl_validateData(const payload& data) const {
	return;
}
void impl_validateDataIndex(size_t index) const {
	return;
}
void impl_validateHeader(std::vector<uint8_t> const& packet) const {
	return;
}

}; // namespace USBCANBridge

}