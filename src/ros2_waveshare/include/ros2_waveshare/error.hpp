/**
 * @file error.hpp
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @brief Error codes for USBCANBridge to be used as template parameter.
 * @version 0.1
 * @date 2025-09-11
 */

#include <system_error>
#include <string>

namespace USBCANBridge {

/**
 * @enum Status
 * @brief Enumeration of error codes for USBCANBridge operations.
 */
enum class Status
{
	SUCCESS = 0,
	WBAD_START = 1,
	WBAD_TYPE = 2,
	WBAD_LENGTH = 3,
	WBAD_ID = 4,
	WBAD_DATA = 5,
	WBAD_DLC = 6,
	WBAD_FORMAT = 7,
	WBAD_CHECKSUM = 8,
	WTIMEOUT = 9,
	DNOT_FOUND = 10,
	DNOT_OPEN = 11,
	DALREADY_OPEN = 12,
	DREAD_ERROR = 13,
	DWRITE_ERROR = 14,
	DCONFIG_ERROR = 15,
	CAN_SDO_TIMEOUT = 16,
	CAN_SDO_ABORT = 17,
	CAN_PDO_ERROR = 18,
	CAN_NMT_ERROR = 19,
	UNKNOWN = 255
};

/**
 * @class USBCANErrorCategory
 * @brief Custom error category for USBCANBridge errors.
 */
class USBCANErrorCategory : public std::error_category
{
public:
const char* name() const noexcept override
{
	return "USBCANBridge::Status";
}

std::string message(int ev) const override
{
	switch (static_cast<Status>(ev))
	{
	case Status::SUCCESS:
		return "Success";
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

// Make error_code from Status
std::error_code make_error_code(Status e)
{
	return { static_cast<int>(e), usbcan_category() };
}

} // namespace USBCANBridge

// Register the enum for use with std::error_code
namespace std
{
template <>
struct is_error_code_enum<USBCANBridge::Status> : true_type
{
};
} // namespace std

/**
 * Usage example:
 * #include "ros2_waveshare/error.hpp"
 *
 * std::error_code ec = USBCANBridge::Status::DNOT_FOUND;
 * if (ec) {
 *     std::cerr << "Error: " << ec.message() << std::endl;
 * }
 */
