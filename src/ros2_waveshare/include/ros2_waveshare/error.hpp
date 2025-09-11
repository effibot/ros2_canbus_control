/**
 * @file error.hpp
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @brief Error codes for USBCANbridge to be used as template parameter.
 * @version 0.1
 * @date 2025-09-11
 */

#pragma once

#include <cstdint>

namespace USBCANbridge {
enum class Status : std::uint8_t {
	// * Waveshare USB-CAN adapter error codes
	WBAD_TYPE = 1,
	WBAD_LENGTH = 2,
	WBAD_ID = 3,
	WBAD_DATA = 4,
	WBAD_FORMAT = 5,
	WBAD_CHECKSUM = 6,
	WTIMEOUT = 7,

	// * Device error codes
	DNOT_FOUND = 1,
	DNOT_OPEN = 2,
	DALREADY_OPEN = 3,
	DREAD_ERROR = 4,
	DWRITE_ERROR = 5,
	DCONFIG_ERROR = 6,

	// * CANopen protocol error codes

	CAN_SDO_TIMEOUT = 1,
	CAN_SDO_ABORT = 2,
	CAN_PDO_ERROR = 3,
	CAN_NMT_ERROR = 4,

	// * Generic error codes
	SUCCESS = 0,
	UNKNOWN = 255
};

} // namespace USBCANbridge