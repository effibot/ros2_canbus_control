/**
 * @file common.hpp
 * @brief Common utilities and definitions for USB-CAN adapter frames.
 *
 * This file provides shared enums, constants, and helper functions used across
 * different frame implementations for USB-CAN bridge communication. It defines
 * the protocol constants, frame types, baud rates, and communication modes
 * supported by the USB-CAN adapter.
 *
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @version 1.0
 * @date 2025-09-11
 * @copyright Copyright (c) 2025
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

/**
 * @namespace USBCANBridge
 * @brief Namespace containing all USB-CAN bridge related functionality.
 *
 * This namespace encapsulates all classes, enums, and functions related to
 * USB-CAN adapter communication, providing a clean separation from other
 * system components.
 */
namespace USBCANBridge {

/**
 * @brief Converts an enum value to std::byte.
 *
 * This template function safely converts any enum type to std::byte by first
 * casting to the underlying type and then to std::byte. It's useful for
 * serializing enum values into byte arrays for USB-CAN frame construction.
 *
 * @tparam EnumType The enum type to convert (must be an enum class)
 * @param value The enum value to convert
 * @return std::byte representation of the enum value
 *
 * @example
 * @code
 * auto start_byte = to_byte(Constants::START_BYTE);
 * auto type_byte = to_byte(Type::DATA_FIXED);
 * @endcode
 */
template<typename EnumType>
constexpr std::byte to_byte(EnumType value) {
	return static_cast<std::byte>(static_cast<std::underlying_type_t<EnumType> >(value));
}

/**
 * @brief Converts a std::byte to an enum value.
 *
 * This template function safely converts std::byte back to an enum type by
 * first casting to the underlying type and then to the enum. It's useful for
 * deserializing byte arrays back into meaningful enum values when parsing
 * received USB-CAN frames.
 *
 * @tparam EnumType The enum type to convert to (must be an enum class)
 * @param value The std::byte value to convert
 * @return EnumType representation of the byte value
 *
 * @warning Ensure the byte value corresponds to a valid enum value to avoid
 *          undefined behavior.
 *
 * @example
 * @code
 * std::byte received_byte = std::byte{0xAA};
 * auto constant = from_byte<Constants>(received_byte);
 * if (constant == Constants::START_BYTE) {
 *     // Process start of frame
 * }
 * @endcode
 */
template<typename EnumType>
constexpr EnumType from_byte(std::byte value) {
	return static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType> >(value));
}

/**
 * @brief Protocol constants for USB-CAN frame structure.
 *
 * These constants define the fixed byte values used in the USB-CAN protocol
 * for frame delimiting and structure identification. All frames must use
 * these specific values at designated positions to ensure proper communication.
 */
enum class Constants : std::uint8_t {
	START_BYTE = 0xAA,    ///< Frame start delimiter byte - marks beginning of frame
	MSG_HEADER = 0x55,    ///< Message header identifier - follows start byte
	END_BYTE = 0x55,      ///< Frame end delimiter byte - marks end of frame
	RESERVED0 = 0x00      ///< Reserved byte value - always set to 0x00
};

/**
 * @brief Frame type classification for USB-CAN messages.
 *
 * Defines the different types of frames that can be sent through the USB-CAN
 * adapter, distinguishing between data transmission and configuration commands,
 * and between fixed-length and variable-length frame formats.
 */
enum class Type : std::uint8_t {
	DATA_FIXED = 0x01,    ///< Fixed-length data frame (20 bytes total)
	DATA_VAR = 0xC0,      ///< Variable-length data frame (up to 8 data bytes)
	CONF_FIXED = 0x02,    ///< Fixed-length configuration frame
	CONF_VAR = 0x12       ///< Variable-length configuration frame
};

/**
 * @brief CAN frame identifier format specification.
 *
 * Specifies whether the CAN frame uses standard (11-bit) or extended (29-bit)
 * identifier format, combined with the frame structure type (fixed or variable).
 * This affects how the CAN ID is interpreted and transmitted on the bus.
 */
enum class FrameType : std::uint8_t {
	STD_FIXED = 0x01,     ///< Standard ID (11-bit) with fixed frame structure
	STD_VAR = 0,          ///< Standard ID (11-bit) with variable frame structure
	EXT_FIXED = 0x02,     ///< Extended ID (29-bit) with fixed frame structure
	EXT_VAR = 1           ///< Extended ID (29-bit) with variable frame structure
};

/**
 * @brief CAN frame format type (data vs remote).
 *
 * Distinguishes between data frames (carrying actual payload data) and remote
 * transmission request frames (requesting data from another node). The format
 * also indicates whether using fixed or variable frame structure.
 */
enum class FrameFmt : std::uint8_t {
	DATA_FIXED = 0x01,    ///< Data frame with fixed structure (carries data payload)
	REMOTE_FIXED = 0x02,  ///< Remote transmission request with fixed structure
	DATA_VAR = 0,         ///< Data frame with variable structure
	REMOTE_VAR = 1        ///< Remote transmission request with variable structure
};

/**
 * @brief CAN bus baud rate configuration.
 *
 * Defines the supported baud rates for CAN bus communication. The choice of
 * baud rate affects both communication speed and maximum cable length. Higher
 * speeds provide faster data transfer but limit the maximum distance between nodes.
 *
 * @note Cable length limitations (approximate):
 * - 1 Mbps: up to 25m
 * - 500 kbps: up to 100m
 * - 250 kbps: up to 250m
 * - 125 kbps: up to 500m
 * - Lower speeds: up to 1000m+
 */
enum class CANBaud : std::uint8_t {
	SPEED_1000K = 0x01,   ///< 1 Mbps - Highest speed, shortest distance (~25m max)
	SPEED_800K = 0x02,    ///< 800 kbps - High speed operation
	SPEED_500K = 0x03,    ///< 500 kbps - Common industrial standard (~100m max)
	SPEED_400K = 0x04,    ///< 400 kbps - High speed with moderate distance
	SPEED_250K = 0x05,    ///< 250 kbps - Good balance of speed and distance (~250m max)
	SPEED_200K = 0x06,    ///< 200 kbps - Medium speed operation
	SPEED_125K = 0x07,    ///< 125 kbps - Common automotive standard (~500m max)
	SPEED_100K = 0x08,    ///< 100 kbps - Lower speed, longer distance
	SPEED_50K = 0x09,     ///< 50 kbps - Long distance applications
	SPEED_20K = 0x0A,     ///< 20 kbps - Very long distance
	SPEED_10K = 0x0B,     ///< 10 kbps - Maximum distance applications
	SPEED_5K = 0x0C       ///< 5 kbps - Extreme distance, lowest speed (~1000m+ max)
};

/**
 * @brief Serial communication baud rates.
 *
 * Defines the supported baud rates for serial communication between the host
 * computer and the USB-CAN adapter. Higher rates enable faster data transfer
 * but may be limited by cable quality and length.
 *
 * @note Choose the highest rate that provides stable communication for your setup.
 */
enum class SerialBaud : std::uint32_t {
	BAUD_9600 = 9600,         ///< 9600 bps - Basic communication rate, most compatible
	BAUD_19200 = 19200,       ///< 19200 bps - Low speed, high compatibility
	BAUD_38400 = 38400,       ///< 38400 bps - Moderate speed
	BAUD_57600 = 57600,       ///< 57600 bps - Higher speed operation
	BAUD_115200 = 115200,     ///< 115200 bps - Common high-speed rate
	BAUD_1228800 = 1228800,   ///< 1.2288 Mbps - Very high speed (may need quality cables)
	BAUD_200000 = 2000000     ///< 2 Mbps - Maximum supported speed (short cables only)
};

/**
 * @brief CAN controller operating modes.
 *
 * Defines the different operating modes for the CAN controller. These modes
 * are primarily used for testing, debugging, and special operational requirements.
 * Normal mode should be used for standard CAN bus communication.
 */
enum class CANMode : std::uint8_t {
	NORMAL = 0x00,            ///< Normal CAN operation mode - standard bus communication
	LOOPBACK = 0x01,          ///< Loopback mode - transmitted frames are received back internally
	SILENT = 0x02,            ///< Silent mode - can receive frames but cannot transmit (listen-only)
	LOOPBACK_SILENT = 0x03    ///< Combined loopback and silent mode - for testing without bus impact
};

/**
 * @brief Automatic retransmission control.
 *
 * Controls whether the CAN controller automatically retransmits frames
 * that encounter errors, collisions, or arbitration loss on the bus.
 * Disabling automatic retransmission can be useful for time-critical applications.
 */
enum class RTX : std::uint8_t {
	AUTO = 0x00,              ///< Automatic retransmission enabled (standard CAN behavior)
	OFF = 0x01                ///< Automatic retransmission disabled (single-shot mode)
};

/**
 * @brief Byte position indices for fixed-size USB-CAN frames.
 *
 * Defines the byte positions within a fixed-size (20-byte) USB-CAN frame structure.
 * This enum provides a convenient and type-safe way to access specific bytes in the
 * frame without using magic numbers, improving code readability and maintainability.
 *
 * @note Frame structure (20 bytes total):
 * [START][HEADER][TYPE][FRAME_TYPE][FRAME_FMT][ID0-ID3][DLC][DATA0-DATA7][RESERVED][CHECKSUM]
 */
enum class FixedSizeIndex : std::size_t {
	START = 0,        ///< Position of start delimiter byte (0xAA)
	HEADER = 1,       ///< Position of header byte (0x55)
	TYPE = 2,         ///< Position of frame type byte (Type enum)
	FRAME_TYPE = 3,   ///< Position of CAN frame type byte (FrameType enum)
	FRAME_FMT = 4,    ///< Position of frame format byte (FrameFmt enum)
	ID_0 = 5,         ///< Position of CAN ID byte 0 (least significant byte)
	ID_1 = 6,         ///< Position of CAN ID byte 1
	ID_2 = 7,         ///< Position of CAN ID byte 2
	ID_3 = 8,         ///< Position of CAN ID byte 3 (most significant byte)
	DLC = 9,          ///< Position of data length code byte (0-8)
	DATA_0 = 10,      ///< Position of data byte 0
	DATA_1 = 11,      ///< Position of data byte 1
	DATA_2 = 12,      ///< Position of data byte 2
	DATA_3 = 13,      ///< Position of data byte 3
	DATA_4 = 14,      ///< Position of data byte 4
	DATA_5 = 15,      ///< Position of data byte 5
	DATA_6 = 16,      ///< Position of data byte 6
	DATA_7 = 17,      ///< Position of data byte 7
	RESERVED = 18,    ///< Position of reserved byte (always 0x00)
	CHECKSUM = 19     ///< Position of checksum byte (frame validation)
};

} // namespace USBCANBridge