#pragma once

#include <cstdint>

namespace USBCANBridge
{

//* -- USB adapter constants and enums --*
// Basic definitions for common and constant values
enum class Constants : uint8_t
{
	START_BYTE = 0xAA,
	MSG_HEADER = 0x55,
	DEF_FILTER0 = 0x00,
	DEF_FILTER1 = 0x00,
	DEF_FILTER2 = 0x00,
	DEF_FILTER3 = 0x00,
	DEF_MASK0 = 0x00,
	DEF_MASK1 = 0x00,
	DEF_MASK2 = 0x00,
	DEF_MASK3 = 0x00,
	RESERVED0 = 0x00,
	RESERVED1 = 0x00,
	RESERVED2 = 0x00,
	RESERVED3 = 0x00,
	END_BYTE = 0x55,
	VAR_BASE = 0xC0
};

//? Adapter frame setting type enum
enum class SettingFrameType : uint8_t
{
	FIXED = 0x02,
	VARIABLE = 0x12
};

enum class FrameType : uint8_t {
	STD_VAR = 0,      // this is a bit
	EXT_VAR = 1,      // this is a bit
	STD_FIXED = 0x01, // this is a byte
	EXT_FIXED = 0x02, // this is a byte
};

//? Adapter frame format enum
enum class FrameFmt : uint8_t
{
	STD = 0x01,
	EXT = 0x02
};

enum class FrameFmtVar : uint8_t
{
	DATA = 0,
	REMOTE = 1
};

//? Adapter CAN speed enum
enum class CANBaud : uint8_t
{
	SPEED_1000K = 0x01,
	SPEED_800K = 0x02,
	SPEED_500K = 0x03,
	SPEED_400K = 0x04,
	SPEED_250K = 0x05,
	SPEED_200K = 0x06,
	SPEED_125K = 0x07,
	SPEED_100K = 0x08,
	SPEED_50K = 0x09,
	SPEED_20K = 0x0A,
	SPEED_10K = 0x0B,
	SPEED_5K = 0x0C
};

//? Serial port baud rate enum
enum class SerialBaud : uint32_t
{
	BAUD_9600 = 9600,
	BAUD_19200 = 19200,
	BAUD_38400 = 38400,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200,
	BAUD_1228800 = 1228800,
	BAUD_200000 = 2000000
};

//? Adapter operating mode enum
enum class CANMode : uint8_t
{
	NORMAL = 0x00,
	LOOPBACK = 0x01,
	SILENT = 0x02,
	LOOPBACK_SILENT = 0x03
};

//? Adapter retransmission enum
enum class RTX : uint8_t
{
	AUTO = 0x00,
	OFF = 0x01
};

enum class FixedSizeIndex : uint8_t
{
	TYPE = 0,
	FRAME_TYPE = 1,
	FRAME_FMT = 2,
	ID_0 = 3,
	ID_1 = 4,
	ID_2 = 5,
	ID_3 = 6,
	DLC = 7,
	DATA_0 = 8,
	DATA_1 = 9,
	DATA_2 = 10,
	DATA_3 = 11,
	DATA_4 = 12,
	DATA_5 = 13,
	DATA_6 = 14,
	DATA_7 = 15,
	RESERVED = 16,
	CHECKSUM = 17
};

enum class VariableSizeIndex : uint8_t
{
	TYPE = 0,
	ID_START = 1, // ID starts here (2 or 4 bytes)
	// ID_END is dynamic based on ID size
	DATA_START = 3, // Data starts here (after ID)
	// DATA_END is dynamic based on DLC
	END = 10 // End byte position (after data)
};

// Utility functions to avoid static_cast repetition (C++17 style)
[[nodiscard]] constexpr auto to_uint8(Constants value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(SettingFrameType value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(FrameType value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(FrameFmt value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(FrameFmtVar value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(CANBaud value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint32(SerialBaud value) noexcept {
	return static_cast<uint32_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(CANMode value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(RTX value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(FixedSizeIndex value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(VariableSizeIndex value) noexcept {
	return static_cast<uint8_t>(value);
}

// Additional convenience functions for common operations (C++17 style)
[[nodiscard]] constexpr bool is_standard_frame(FrameType type) noexcept {
	return type == FrameType::STD_FIXED || type == FrameType::STD_VAR;
}

[[nodiscard]] constexpr bool is_extended_frame(FrameType type) noexcept {
	return type == FrameType::EXT_FIXED || type == FrameType::EXT_VAR;
}

[[nodiscard]] constexpr bool is_fixed_frame(FrameType type) noexcept {
	return type == FrameType::STD_FIXED || type == FrameType::EXT_FIXED;
}

[[nodiscard]] constexpr bool is_variable_frame(FrameType type) noexcept {
	return type == FrameType::STD_VAR || type == FrameType::EXT_VAR;
}

// Helper function to create variable frame type byte (C++17 style)
[[nodiscard]] constexpr auto make_variable_type_byte(FrameFmt frame_fmt, FrameFmtVar frame_fmt_var, uint8_t dlc) noexcept {
	return to_uint8(Constants::VAR_BASE) |
	       (frame_fmt == FrameFmt::EXT ? 0x20 : 0) |
	       (frame_fmt_var == FrameFmtVar::REMOTE ? 0x10 : 0) |
	       (dlc & 0x0F);
}

//? defines of default settings
constexpr uint32_t DEF_FILTER_SETTING = 0x00000000;                    // Default filter setting (accept all IDs)
constexpr uint32_t DEF_MASK_SETTING = 0x00000000;                      // Default mask setting (no bits are masked)
constexpr FrameType DEF_FRAME_FORMAT = FrameType::STD_VAR;             // Default frame format for the adapter
constexpr FrameFmt DEF_FRAME_TYPE = FrameFmt::STD;                     // Default frame type for the adapter
constexpr CANBaud DEF_CAN_SPEED = CANBaud::SPEED_1000K;                // Default CAN bus speed
constexpr SerialBaud DEF_BAUD_RATE = SerialBaud::BAUD_200000;          // Default baud rate for the serial interface
constexpr CANMode DEF_CAN_MODE = CANMode::NORMAL;                      // Default operating mode for the adapter
constexpr RTX DEF_RTX = RTX::AUTO;                                     // Default auto-retransmission setting

} // namespace USBCANBridge