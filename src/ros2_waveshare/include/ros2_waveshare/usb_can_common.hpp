/**
 * @file usb_can_common.hpp
 * @brief Common protocol definitions, enumerations and helper utilities for USB-CAN adapter frames.
 *
 * This header centralizes protocol constants, strongly-typed enumerations, index helpers for
 * fixed and variable frame layouts, as well as constexpr conversion helpers and utility bit
 * manipulation routines. All frame implementation headers include this file to ensure a single
 * authoritative definition of protocol semantics.
 *
 * Design goals:
 * - Strong typing via enum class to avoid accidental mixing of domains (e.g., Type vs FrameFmt)
 * - constexpr inline helpers to eliminate repetitive static_cast usage
 * - Clear Doxygen documentation to generate complete API reference
 * - Arithmetic operator overloads for VarSizeIndex to simplify dynamic layout computation
 *
 * @author Andrea
 * @date 2025
 */
#pragma once

#include <cstdint>
#include <stdexcept>
#include <cstddef>

namespace USBCANBridge
{

//* -- USB adapter constants and enums --*
// Basic definitions for common and constant values
/**
 * @enum Constants
 * @brief Fundamental single-byte protocol constants used in frame construction.
 *
 * These values appear verbatim inside serialized frames as structural markers
 * (e.g., START_BYTE, END_BYTE) or reserved padding. They are not meant to be
 * mutated at runtime.
 */
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
};

/**
 * @enum Type
 * @brief Adapter packet type (distinguishes fixed vs variable and data vs config).
 */
enum class Type : uint8_t {
	/**
	 * @brief USB-CAN-A adapter packet type enumeration
	 * This enum defines the different packet types supported by the USB-CAN-A adapter.
	 * For the fixed 20 bytes packet, the type is determined by byte 2 and can be:
	 * - {0x01} for Data packet,
	 * - {0x02} for Config packet and 20-byte protocol.
	 * - {0x13} for Config packet and variable length protocol.
	 * For the variable length packet, the type is determined by bits 6-7 of the type byte
	 * and is always {0xC0} (bits 7 and 6 set to 1).
	 * @note Two adapters cannot communicate if they use different protocol types
	 */
	DATA_FIXED = 0x01,
	DATA_VAR = 0xC0,
	CONF_FIXED = 0x02,
	CONF_VAR = 0x12,
};

/**
 * @enum FrameType
 * @brief CAN identifier width specifier for each protocol variant.
 */
enum class FrameType : uint8_t {
	/**
	 * @brief USB-CAN-A adapter frame type enumeration
	 * This enum defines the frame type for frames used by the USB-CAN-A adapter.
	 * For the fixed 20 bytes packet, the frame type is determined by byte 3 and can be:
	 * - {0x01} for Standard Frame (ID in 2 bytes)
	 * - {0x02} for Extended Frame (ID in 4 bytes)
	 * For the variable length packet, the frame type is determined by bit 5 of the type byte:
	 * - {0} for Standard Frame (ID in 2 bytes)
	 * - {1} for Extended Frame (ID in 4 bytes)
	 * This enum is used in conjunction with FrameFmt to fully define the frame structure.
	 */
	STD_FIXED = 0x01,
	STD_VAR = 0,
	EXT_FIXED = 0x02,
	EXT_VAR = 1,
};

/**
 * @enum FrameFmt
 * @brief CAN frame format (data payload or remote request) in both fixed and variable protocols.
 */
enum class FrameFmt : uint8_t {
	/**
	 * @brief USB-CAN-A adapter frame format enumeration
	 * This enum defines the frame format for frames used by the USB-CAN-A adapter.
	 * For the fixed 20 bytes packet, the frame format is determined by byte 4 and can be:
	 * - {0x01} for Data Frame
	 * - {0x02} for Remote Frame
	 * For the variable length packet, the frame format is determined by bit 4 of the type byte:
	 * - {0} for Data Frame
	 * - {1} for Remote Frame
	 * This enum is used in conjunction with FrameType to fully define the frame structure.
	 */

	DATA_FIXED = 0x01,
	REMOTE_FIXED = 0x02,
	DATA_VAR = 0,
	REMOTE_VAR = 1
};

/**
 * @enum CANBaud
 * @brief Enumeration of supported CAN bus nominal bit rates.
 */
enum class CANBaud : uint8_t {
	/**
	 * @brief USB-CAN-A adapter CAN bus speed enumeration
	 * This enum defines the CAN bus speeds supported by the USB-CAN-A adapter.
	 * The speeds are represented by specific byte values used in the adapter configuration.
	 * - SPEED_1000K: 1 Mbps
	 * - SPEED_800K: 800 kbps
	 * - SPEED_500K: 500 kbps
	 * - SPEED_400K: 400 kbps
	 * - SPEED_250K: 250 kbps
	 * - SPEED_200K: 200 kbps
	 * - SPEED_125K: 125 kbps
	 * - SPEED_100K: 100 kbps
	 * - SPEED_50K: 50 kbps
	 * - SPEED_20K: 20 kbps
	 * - SPEED_10K: 10 kbps
	 * - SPEED_5K: 5 kbps
	 * @note Not all speeds may be supported by all CAN networks or devices.
	 */
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

/**
 * @enum SerialBaud
 * @brief Enumeration of supported USB serial bridge baud rates.
 */
enum class SerialBaud : uint32_t {
	/**
	 * @brief USB-CAN-A adapter serial port baud rate enumeration
	 * This enum defines the baud rates supported by the USB-CAN-A adapter for its serial interface.
	 * The baud rates are represented by their actual values in bits per second (bps).
	 * - BAUD_9600: 9600 bps
	 * - BAUD_19200: 19200 bps
	 * - BAUD_38400: 38400 bps
	 * - BAUD_57600: 57600 bps
	 * - BAUD_115200: 115200 bps
	 * - BAUD_1228800: 1228800 bps
	 * - BAUD_200000: 2000000 bps
	 * @note The actual supported baud rates may depend on the specific adapter model and its firmware.
	 * @note Ensure that the baud rate set on the adapter matches the baud rate configured in the host system.
	 * @note Higher baud rates may provide better performance but could be less stable over longer cables or in noisy environments.
	 * @note Default baud rate is 2M and as stated by Waveshare, it is recommended to don't change it.
	 */
	BAUD_9600 = 9600,
	BAUD_19200 = 19200,
	BAUD_38400 = 38400,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200,
	BAUD_1228800 = 1228800,
	BAUD_200000 = 2000000
};

/**
 * @enum CANMode
 * @brief Operating modes controlling transceiver behavior and echo.
 */
enum class CANMode : uint8_t {
	/**
	 * @brief USB-CAN-A adapter operating mode enumeration
	 * This enum defines the operating modes supported by the USB-CAN-A adapter.
	 * The modes determine how the adapter interacts with the CAN bus.
	 * - NORMAL: Normal mode, the adapter can send and receive messages on the CAN bus
	 * - LOOPBACK: Loopback mode, the adapter only sends messages to itself for testing
	 * - SILENT: Silent mode, the adapter only listens to the CAN bus without sending
	 * - LOOPBACK_SILENT: Loopback silent mode, the adapter listens to its own messages without sending them to the CAN bus
	 * @note The mode should be set according to the intended use case of the adapter.
	 */
	NORMAL = 0x00,
	LOOPBACK = 0x01,
	SILENT = 0x02,
	LOOPBACK_SILENT = 0x03
};

/**
 * @enum RTX
 * @brief Auto retransmission policy for failed CAN frame arbitration or errors.
 */
enum class RTX : uint8_t {
	/**
	 * @brief USB-CAN-A adapter auto-retransmission setting enumeration
	 * This enum defines the auto-retransmission settings supported by the USB-CAN-A adapter.
	 * The settings determine whether the adapter will automatically retransmit messages that fail to be sent on the CAN bus.
	 * - AUTO: Auto-retransmission enabled, the adapter will automatically retry sending failed messages
	 * - OFF: Auto-retransmission disabled, the adapter will not retry sending failed messages
	 * @note Enabling auto-retransmission can improve message delivery reliability but may increase bus traffic.
	 * @note Disabling auto-retransmission can reduce bus traffic but may lead to lost messages.
	 */
	AUTO = 0x00,
	OFF = 0x01
};

/**
 * @enum FixedSizeIndex
 * @brief Byte offsets for the 20-byte fixed frame representation.
 */
enum class FixedSizeIndex : std::size_t {
	/**
	 * @brief USB-CAN-A adapter fixed frame byte index enumeration
	 * This enum defines the byte indices for the fixed 20-byte frame format used by the USB-CAN-A adapter.
	 * The indices correspond to specific fields within the frame, allowing for easy access and manipulation of the frame data.
	 * The complete frame consists of:
	 * - [0] Start byte (always {0xAA}) : inherited from base
	 * - [1] Message header byte (always {0x55}) : inherited from base
	 * - [2] Type byte (always {0x01})
	 * - [3] Frame type byte ({0x01} for standard, {0x02} for extended)
	 * - [4] Framework Format byte ({0x01} for data frame, {0x02} for remote frame)
	 * - [5-8] Frame ID [4] bytes, high bytes at the front, low bytes at the end (little-endian)
	 * - [9] Frame Data Length Code (DLC) byte (0-8)
	 * - [10-17] Frame Data [8] bytes (padded with zeros if DLC < 8)
	 * - [18] Reserved byte (always {0x00})
	 * - [19] Checksum Code (The low 8 bits of the cumulative sum from frame type to error code )
	 * @note This enum is specifically for the fixed frame format and may not apply to variable length frames.
	 */
	START = 0,
	HEADER = 1,
	TYPE = 2,
	FRAME_TYPE = 3,
	FRAME_FMT = 4,
	ID_0 = 5,
	ID_1 = 6,
	ID_2 = 7,
	ID_3 = 8,
	DLC = 9,
	DATA_0 = 10,
	DATA_1 = 11,
	DATA_2 = 12,
	DATA_3 = 13,
	DATA_4 = 14,
	DATA_5 = 15,
	DATA_6 = 16,
	DATA_7 = 17,
	RESERVED = 18,
	CHECKSUM = 19
};

/**
 * @enum VarSizeIndex
 * @brief Base byte offsets for the variable-length frame representation.
 *
 * Dynamic indices (ID_END, DATA_START, DATA_END, END) are computed at runtime and
 * stored in the variable frame object; this enum supplies the starting anchors.
 */
enum class VarSizeIndex : std::size_t {
	/**
	 * @brief USB-CAN-A adapter variable frame byte index enumeration
	 * This enum defines the byte indices for the variable length frame format used by the USB-CAN-A adapter.
	 * The indices correspond to specific fields within the frame, allowing for easy access and manipulation of the frame data.
	 * The complete frame consists of:
	 * - Start byte (always {0xAA}) : inherited from base
	 * - Type byte:
	 * 	- [Type header] bit 6-7: Always 0xC0 (bits 7 and 6 set to 1)
	 * 	- [Frame type] bit 5: {0} for standard, {1} for extended
	 * 	- [Frame format] bit 4: {0} for data frame, {1} for remote frame
	 * 	- [DLC] bits 0-3: Data Length Code (0-8)
	 * - Frame ID [2 or 4] bytes, high bytes at the front, low bytes at the end (little-endian)
	 * - Frame Data [0-8] bytes (present only if DLC > 0)
	 * - End byte (always {0x55}) : inherited from base
	 * @note This enum is specifically for the variable length frame format and may not apply to fixed frames.
	 */
	START = 0,
	TYPE_HEADER = 1,
	FRAME_TYPE = 2,
	FRAME_FMT = 3,
	DLC = 4,
	ID_START = 5,
	// ID_END is dynamic based on ID size
	// DATA_START is dynamic based on ID size
	// DATA_END is dynamic based on ID size
	// END is dynamic based on ID size
};

// -------------------------------------------------------------------------------------------------
// Conversion helpers
// -------------------------------------------------------------------------------------------------
/**
 * @name Conversion helpers
 * @brief constexpr wrappers replacing repetitive static_cast expressions.
 * @note All functions are noexcept and eligible for compile-time evaluation.
 * @{ */
// Utility functions to avoid static_cast repetition (C++17 style)
[[nodiscard]] constexpr auto to_uint8(Constants value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(Type value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(FrameType value) noexcept {
	return static_cast<uint8_t>(value);
}

[[nodiscard]] constexpr auto to_uint8(FrameFmt value) noexcept {
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

[[nodiscard]] constexpr auto to_uint(FixedSizeIndex value) noexcept {
	return static_cast<std::size_t>(value);
}

[[nodiscard]] constexpr auto to_uint(VarSizeIndex value) noexcept {
	return static_cast<std::size_t>(value);
}

[[nodiscard]] constexpr VarSizeIndex to_VarSizeIndex(std::size_t value) noexcept {
	return static_cast<VarSizeIndex>(value);
}
/** @} */

// -------------------------------------------------------------------------------------------------
// VarSizeIndex arithmetic operators
// -------------------------------------------------------------------------------------------------
/**
 * @name VarSizeIndex arithmetic
 * @brief Convenience overloads to manipulate dynamic variable frame indices.
 *
 * These operators intentionally mirror pointer/size_t arithmetic semantics to simplify
 * calculations when recomputing layout boundaries after ID or payload resizing.
 * @{ */
// redefine math operators for VarSizeIndex, so that we can do things like ID_END = ID_START + 1
constexpr VarSizeIndex operator+(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return static_cast<VarSizeIndex>(to_uint(lhs) + rhs);
}

constexpr VarSizeIndex operator+(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return static_cast<VarSizeIndex>(lhs + to_uint(rhs));
}

constexpr VarSizeIndex operator-(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return static_cast<VarSizeIndex>(to_uint(lhs) - rhs);
}

constexpr VarSizeIndex operator-(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return static_cast<VarSizeIndex>(lhs - to_uint(rhs));
}

constexpr VarSizeIndex& operator+=(VarSizeIndex& lhs, std::size_t rhs) noexcept {
	lhs = lhs + rhs;
	return lhs;
}

constexpr VarSizeIndex& operator-=(VarSizeIndex& lhs, std::size_t rhs) noexcept {
	lhs = lhs - rhs;
	return lhs;
}

constexpr bool operator<(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return to_uint(lhs) < rhs;
}

constexpr bool operator<(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return lhs < to_uint(rhs);
}

constexpr bool operator<=(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return to_uint(lhs) <= rhs;
}

constexpr bool operator<=(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return lhs <= to_uint(rhs);
}

constexpr bool operator>(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return to_uint(lhs) > rhs;
}

constexpr bool operator>(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return lhs > to_uint(rhs);
}

constexpr bool operator>=(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return to_uint(lhs) >= rhs;
}

constexpr bool operator>=(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return lhs >= to_uint(rhs);
}

constexpr bool operator==(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return to_uint(lhs) == rhs;
}

constexpr bool operator==(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return lhs == to_uint(rhs);
}

constexpr bool operator!=(VarSizeIndex lhs, std::size_t rhs) noexcept {
	return to_uint(lhs) != rhs;
}

constexpr bool operator!=(std::size_t lhs, VarSizeIndex rhs) noexcept {
	return lhs != to_uint(rhs);
}
/** @} */



// -------------------------------------------------------------------------------------------------
// Variable frame type byte helpers
// -------------------------------------------------------------------------------------------------
/**
 * @brief Assemble a packed type byte for variable-length frames.
 *
 * Layout (bit positions):
 * - 7..6: constant 1 bits (0b11)
 * - 5: frame type (0 = standard, 1 = extended)
 * - 4: frame format (0 = data, 1 = remote)
 * - 3..0: DLC (0..8)
 *
 * @throws std::out_of_range if dlc > 8
 */
// Helper function to create variable frame type byte (C++17 style)
[[nodiscard]] constexpr auto make_variable_type_byte(FrameType type, FrameFmt fmt, uint8_t dlc) {
	/**
	 * @brief Create the type byte for a variable length USB-CAN-A adapter frame
	 * This function constructs the type byte for a variable length frame based on the provided
	 * frame type, frame format, and data length code (DLC).
	 * The type byte is structured as follows:
	 * - Bits 6-7: Always set to 1 (0xC0)
	 * - Bit 5: Frame type (0 for standard, 1 for extended)
	 * - Bit 4: Frame format (0 for data frame, 1 for remote frame)
	 * - Bits 0-3: Data Length Code (DLC) (0-8)
	 * @param type The frame type (standard or extended)
	 * @param fmt The frame format (data frame or remote frame)
	 * @param dlc The data length code (0-8)
	 * @return The constructed type byte for the variable length frame
	 * @note The DLC should be in the range of 0 to 8, as per CAN protocol specifications.
	 */

	if (dlc > 8) {
		throw std::out_of_range("DLC is out of range for CAN frame (0-8)");
	}

	return to_uint8(Type::DATA_VAR) |
	       ((type == FrameType::STD_VAR ? to_uint8(FrameType::STD_VAR) : to_uint8(FrameType::EXT_VAR)) << 5) |
	       ((fmt == FrameFmt::DATA_VAR ? to_uint8(FrameFmt::DATA_VAR) : to_uint8(FrameFmt::REMOTE_VAR)) << 4) |
	       (dlc & 0x0F);
}

[[nodiscard]] constexpr auto parse_variable_type_byte(uint8_t type_byte, FrameType& out_type, FrameFmt& out_fmt, uint8_t& out_dlc) {
	/**
	 * @brief Parse the type byte of a variable length USB-CAN-A adapter frame
	 * This function extracts the frame type, frame format, and data length code (DLC)
	 * from the provided type byte of a variable length frame.
	 * The type byte is structured as follows:
	 * - Bits 6-7: Always set to 1 (0xC0)
	 * - Bit 5: Frame type (0 for standard, 1 for extended)
	 * - Bit 4: Frame format (0 for data frame, 1 for remote frame)
	 * - Bits 0-3: Data Length Code (DLC) (0-8)
	 * @param type_byte The type byte to parse
	 * @param out_type Reference to store the extracted frame type
	 * @param out_fmt Reference to store the extracted frame format
	 * @param out_dlc Reference to store the extracted data length code
	 * @note The DLC should be in the range of 0 to 8, as per CAN protocol specifications.
	 */

	if ((type_byte & 0xC0) != to_uint8(Type::DATA_VAR)) {
		throw std::invalid_argument("Invalid type byte for variable length frame");
	}

	out_type = (type_byte & 0x20) ? FrameType::EXT_VAR : FrameType::STD_VAR;
	out_fmt = (type_byte & 0x10) ? FrameFmt::REMOTE_VAR : FrameFmt::DATA_VAR;
	out_dlc = type_byte & 0x0F;

	if (out_dlc > 8) {
		throw std::out_of_range("DLC is out of range for CAN frame (0-8)");
	}
}


/**
 * @todo Consider relocating convenience frame classification helpers (is_data_frame, etc.)
 *       into a dedicated traits header if/when they are implemented to keep this file lean.
 */

// -------------------------------------------------------------------------------------------------
// Default configuration presets
// -------------------------------------------------------------------------------------------------
/** @brief Default acceptance filter (accept all standard & extended IDs). */
constexpr uint32_t DEF_FILTER_SETTING = 0x00000000;
/** @brief Default mask (no bit masked => all bits relevant). */
constexpr uint32_t DEF_MASK_SETTING = 0x00000000;
/** @brief Preferred default CAN bus speed (1 Mbps). */
constexpr CANBaud DEF_CAN_SPEED = CANBaud::SPEED_1000K;
/** @brief Default high performance serial baud rate (2 Mbps). */
constexpr SerialBaud DEF_BAUD_RATE = SerialBaud::BAUD_200000;
/** @brief Default adapter mode (normal bus participation). */
constexpr CANMode DEF_CAN_MODE = CANMode::NORMAL;
/** @brief Default auto-retransmission behavior (enabled). */
constexpr RTX DEF_RTX = RTX::AUTO;

} // namespace USBCANBridge