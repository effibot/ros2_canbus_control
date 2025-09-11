/**
 * @file error.hpp
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @brief Error codes for USBCANBridge to be used as template parameter.
 * @version 0.1
 * @date 2025-09-11
 */

#include <system_error>
#include <string>

/**
 * @enum Status
 * @brief Enumeration of error codes for USBCANBridge operations.
 * @{ */
/**
 * @file error.hpp
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @brief Error codes for USBCANBridge to be used as template parameter.
 * @version 0.1
 * @date 2025-09-11
 */

#include <system_error>
#include <string>

/**
 * @enum Status
 * @brief Enumeration of error codes for USBCANBridge operations.
 * @{ */
enum class Status
{
	// * General success
	// SUCCESS = 0, //! Implicitly defined as 0, no need to actually define it
	// * Waveshare USB-CAN adapter error codes
	WBAD_TYPE = 1,    // Bad message type
	WBAD_LENGTH = 2,  // Bad message length
	WBAD_ID = 3,      // Bad CAN ID
	WBAD_DATA = 4,    // Bad data
	WBAD_FORMAT = 5,  // Bad format
	WBAD_CHECKSUM = 6, // Bad checksum
	WTIMEOUT = 7,     // Timeout
	// * Device error codes
	DNOT_FOUND = 8,    // Device not found
	DNOT_OPEN = 9,     // Device not open
	DALREADY_OPEN = 10, // Device already open
	DREAD_ERROR = 11,  // Device read error
	DWRITE_ERROR = 12, // Device write error
	DCONFIG_ERROR = 13, // Device configuration error
	// * CANopen protocol error codes
	CAN_SDO_TIMEOUT = 14, // CAN SDO timeout
	CAN_SDO_ABORT = 15, // CAN SDO abort
	CAN_PDO_ERROR = 16, // CAN PDO error
	CAN_NMT_ERROR = 17, // CAN NMT error
	// * Generic error codes
	UNKNOWN = 255 // Unknown error
};
/** @} */
namespace
{  // Anonymous namespace
/**
 * @class USBCANErrorCategory
 * @brief Custom error category for USBCANBridge errors.
 * This class provides human-readable messages for each error code defined in the Status enum.
 * It inherits from std::error_category and overrides the name() and message() methods.
 */
class USBCANErrorCategory : public std::error_category
{
public:
const char* name() const noexcept override
{
	return "USBCANBridge";
}

std::string message(int ev) const override
{
	switch (static_cast<Status>(ev))
	{
	case Status::WBAD_TYPE:
		return "Bad message type";
	case Status::WBAD_LENGTH:
		return "Bad message length";
	case Status::WBAD_ID:
		return "Bad CAN ID";
	case Status::WBAD_DATA:
		return "Bad data";
	case Status::WBAD_FORMAT:
		return "Bad format";
	case Status::WBAD_CHECKSUM:
		return "Bad checksum";
	case Status::WTIMEOUT:
		return "Timeout";
	case Status::DNOT_FOUND:
		return "Device not found";
	case Status::DNOT_OPEN:
		return "Device not open";
	case Status::DALREADY_OPEN:
		return "Device already open";
	case Status::DREAD_ERROR:
		return "Device read error";
	case Status::DWRITE_ERROR:
		return "Device write error";
	case Status::DCONFIG_ERROR:
		return "Device configuration error";
	case Status::CAN_SDO_TIMEOUT:
		return "CAN SDO timeout";
	case Status::CAN_SDO_ABORT:
		return "CAN SDO abort";
	case Status::CAN_PDO_ERROR:
		return "CAN PDO error";
	case Status::CAN_NMT_ERROR:
		return "CAN NMT error";
	case Status::UNKNOWN:
		return "Unknown error";
	default:
		return "Unrecognized error";
	}
}
};

// Get the error category instance
const std::error_category& usbcan_category()
{
	static USBCANErrorCategory instance;
	return instance;
}

}  // Anonymous namespace

// Register the enum for use with std::error_code
namespace std
{
template <>
struct is_error_code_enum<Status> : true_type
{
};
}  // namespace std
// Make error_code from Status
std::error_code make_error_code(Status e)
{
	return { static_cast<int>(e), usbcan_category() };
}
std::error_code make_error_code(Status);

/**
 * * To use:
 * #include "ros2_waveshare/error.hpp"
 * std::error_code ec = Status::DNOT_FOUND;
 * if (ec) {
 *     std::cerr << "Error: " << ec.message() << std::endl;
 * }
 * * End usage example
 */

#include <iostream>
int main()
{
	std::error_code ec = Status::DNOT_FOUND;
	if (ec)
	{
		std::cerr << "Error: " << ec.message() << std::endl;
	}
	return 0;
}
/** @} */
namespace
{  // Anonymous namespace
/**
 * @class USBCANErrorCategory
 * @brief Custom error category for USBCANBridge errors.
 * This class provides human-readable messages for each error code defined in the Status enum.
 * It inherits from std::error_category and overrides the name() and message() methods.
 */
class USBCANErrorCategory : public std::error_category
{
public:
const char* name() const noexcept override
{
	return "USBCANBridge";
}

std::string message(int ev) const override
{
	switch (static_cast<Status>(ev))
	{
	case Status::WBAD_TYPE:
		return "Bad message type";
	case Status::WBAD_LENGTH:
		return "Bad message length";
	case Status::WBAD_ID:
		return "Bad CAN ID";
	case Status::WBAD_DATA:
		return "Bad data";
	case Status::WBAD_FORMAT:
		return "Bad format";
	case Status::WBAD_CHECKSUM:
		return "Bad checksum";
	case Status::WTIMEOUT:
		return "Timeout";
	case Status::DNOT_FOUND:
		return "Device not found";
	case Status::DNOT_OPEN:
		return "Device not open";
	case Status::DALREADY_OPEN:
		return "Device already open";
	case Status::DREAD_ERROR:
		return "Device read error";
	case Status::DWRITE_ERROR:
		return "Device write error";
	case Status::DCONFIG_ERROR:
		return "Device configuration error";
	case Status::CAN_SDO_TIMEOUT:
		return "CAN SDO timeout";
	case Status::CAN_SDO_ABORT:
		return "CAN SDO abort";
	case Status::CAN_PDO_ERROR:
		return "CAN PDO error";
	case Status::CAN_NMT_ERROR:
		return "CAN NMT error";
	case Status::UNKNOWN:
		return "Unknown error";
	default:
		return "Unrecognized error";
	}
}
};

// Get the error category instance
const std::error_category& usbcan_category()
{
	static USBCANErrorCategory instance;
	return instance;
}

}  // Anonymous namespace

// Register the enum for use with std::error_code
namespace std
{
template <>
struct is_error_code_enum<Status> : true_type
{
};
}  // namespace std
// Make error_code from Status
std::error_code make_error_code(Status e)
{
	return { static_cast<int>(e), usbcan_category() };
}
std::error_code make_error_code(Status);

/**
 * * To use:
 * #include "ros2_waveshare/error.hpp"
 * std::error_code ec = Status::DNOT_FOUND;
 * if (ec) {
 *     std::cerr << "Error: " << ec.message() << std::endl;
 * }
 * * End usage example
 */