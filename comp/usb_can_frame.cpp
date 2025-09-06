#include "usb_can_frame.hpp"
#include "usb_can_common.hpp"

#include <iostream>
#include <iomanip>
#include <vector>

using namespace USBCANBridge;

// Helper function to print byte data in hex format
void printBytes(const std::vector<uint8_t>& data, const std::string& label) {
	std::cout << label << " (" << data.size() << " bytes): ";
	for (const auto& byte : data) {
		std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
		          << static_cast<int>(byte) << " ";
	}
	std::cout << std::dec << std::endl;
}

int main() {
	std::cout << "=== USB-CAN Frame Structure Demo ===" << std::endl;

	try {
		// ========== Fixed Size Frame Example ==========
		std::cout << "\n--- Fixed Size Frame Example ---" << std::endl;

		// Create a 20-byte fixed frame for standard data frame
		fixedSize fixed_frame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);

		// Display basic frame information
		std::cout << "Fixed Frame Structure:" << std::endl;
		std::cout << "  Frame size: " << fixed_frame.size() << " bytes" << std::endl;
		std::cout << "  Type: 0x" << std::hex << static_cast<int>(to_uint8(fixed_frame.getType())) << std::dec << std::endl;
		std::cout << "  Frame Type: 0x" << std::hex << static_cast<int>(to_uint8(fixed_frame.getFrameType())) << std::dec << std::endl;
		std::cout << "  Frame Format: 0x" << std::hex << static_cast<int>(to_uint8(fixed_frame.getFrameFmt())) << std::dec << std::endl;

		// Set frame properties
		uint32_t can_id = 0x123; // Standard CAN ID
		fixed_frame.setID(can_id);
		fixed_frame.setDLC(8); // 8 bytes of data

		std::cout << "  CAN ID: 0x" << std::hex << fixed_frame.getID() << std::dec << std::endl;
		std::cout << "  DLC: " << static_cast<int>(fixed_frame.getDLC()) << std::endl;

		// Set some sample data using individual byte setting
		for (size_t i = 0; i < 8; ++i) {
			fixed_frame.setData(i, static_cast<uint8_t>(0x10 + i));
		}

		// Display some data bytes
		std::cout << "  Sample data bytes: ";
		for (size_t i = 0; i < 4; ++i) {
			std::cout << "0x" << std::hex << static_cast<int>(fixed_frame.getData(i)) << " ";
		}
		std::cout << std::dec << std::endl;

		// Access frame data using array operators
		std::cout << "  Start byte: 0x" << std::hex << static_cast<int>(fixed_frame[0]) << std::dec << std::endl;
		std::cout << "  Header byte: 0x" << std::hex << static_cast<int>(fixed_frame[1]) << std::dec << std::endl;
		std::cout << "  Type byte: 0x" << std::hex << static_cast<int>(fixed_frame[2]) << std::dec << std::endl;

		// Validate the frame basic checks
		std::cout << "  Valid start: " << (fixed_frame.isValidStart() ? "Yes" : "No") << std::endl;
		std::cout << "  Valid length: " << (fixed_frame.isValidLength() ? "Yes" : "No") << std::endl;
		std::cout << "  Valid ID: " << (fixed_frame.isValidID() ? "Yes" : "No") << std::endl;

		// ========== Variable Size Frame Example ==========
		std::cout << "\n--- Variable Size Frame Example ---" << std::endl;

		// Create a variable length frame for extended data frame
		varSize var_frame(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 4);

		std::cout << "Variable Frame Structure:" << std::endl;
		std::cout << "  Frame size: " << var_frame.size() << " bytes" << std::endl;
		std::cout << "  Type: 0x" << std::hex << static_cast<int>(to_uint8(var_frame.getType())) << std::dec << std::endl;
		std::cout << "  Frame Type: " << (var_frame.getFrameType() == FrameType::EXT_VAR ? "Extended" : "Standard") << std::endl;
		std::cout << "  Frame Format: " << (var_frame.getFrameFmt() == FrameFmt::DATA_VAR ? "Data" : "Remote") << std::endl;
		std::cout << "  DLC: " << static_cast<int>(var_frame.getDLC()) << std::endl;

		// Set frame properties
		uint32_t extended_can_id = 0x12345678; // Extended CAN ID
		var_frame.setID(extended_can_id);
		std::cout << "  CAN ID: 0x" << std::hex << var_frame.getID() << std::dec << std::endl;

		// Set some sample data (4 bytes as specified in DLC)
		for (size_t i = 0; i < 4; ++i) {
			var_frame.setData(i, static_cast<uint8_t>(0xA0 + i));
		}

		// Display some data bytes
		std::cout << "  Sample data bytes: ";
		for (size_t i = 0; i < 4; ++i) {
			std::cout << "0x" << std::hex << static_cast<int>(var_frame.getData(i)) << " ";
		}
		std::cout << std::dec << std::endl;

		// Validate the frame basic checks
		std::cout << "  Valid start: " << (var_frame.isValidStart() ? "Yes" : "No") << std::endl;
		std::cout << "  Valid length: " << (var_frame.isValidLength() ? "Yes" : "No") << std::endl;
		std::cout << "  Valid ID: " << (var_frame.isValidID() ? "Yes" : "No") << std::endl;

		// ========== Protocol Constants Demo ==========
		std::cout << "\n--- Protocol Constants Demo ---" << std::endl;

		std::cout << "USB-CAN Protocol Constants:" << std::endl;
		std::cout << "  Start byte: 0x" << std::hex << static_cast<int>(to_uint8(Constants::START_BYTE)) << std::dec << std::endl;
		std::cout << "  Message header: 0x" << std::hex << static_cast<int>(to_uint8(Constants::MSG_HEADER)) << std::dec << std::endl;
		std::cout << "  End byte: 0x" << std::hex << static_cast<int>(to_uint8(Constants::END_BYTE)) << std::dec << std::endl;

		std::cout << "\nSupported CAN Baud Rates:" << std::endl;
		std::cout << "  1000K: 0x" << std::hex << static_cast<int>(to_uint8(CANBaud::SPEED_1000K)) << std::dec << std::endl;
		std::cout << "  500K: 0x" << std::hex << static_cast<int>(to_uint8(CANBaud::SPEED_500K)) << std::dec << std::endl;
		std::cout << "  250K: 0x" << std::hex << static_cast<int>(to_uint8(CANBaud::SPEED_250K)) << std::dec << std::endl;
		std::cout << "  125K: 0x" << std::hex << static_cast<int>(to_uint8(CANBaud::SPEED_125K)) << std::dec << std::endl;

		std::cout << "\nFrame Type Testing:" << std::endl;

		// Test variable type byte creation
		uint8_t type_byte = make_variable_type_byte(FrameType::STD_VAR, FrameFmt::DATA_VAR, 8);
		std::cout << "  Variable type byte (STD, DATA, DLC=8): 0x" << std::hex << static_cast<int>(type_byte) << std::dec << std::endl;

		type_byte = make_variable_type_byte(FrameType::EXT_VAR, FrameFmt::REMOTE_VAR, 0);
		std::cout << "  Variable type byte (EXT, REMOTE, DLC=0): 0x" << std::hex << static_cast<int>(type_byte) << std::dec << std::endl;

		// Test type byte parsing
		FrameType parsed_type;
		FrameFmt parsed_fmt;
		uint8_t parsed_dlc;

		parse_variable_type_byte(0xC8, parsed_type, parsed_fmt, parsed_dlc); // 0xC8 = STD, DATA, DLC=8
		std::cout << "  Parsed type byte 0xC8: "
		          << (parsed_type == FrameType::STD_VAR ? "STD" : "EXT") << ", "
		          << (parsed_fmt == FrameFmt::DATA_VAR ? "DATA" : "REMOTE") << ", "
		          << "DLC=" << static_cast<int>(parsed_dlc) << std::endl;

		// ========== Index Enumeration Demo ==========
		std::cout << "\n--- Index Enumeration Demo ---" << std::endl;

		std::cout << "Fixed Frame Indices:" << std::endl;
		std::cout << "  START: " << to_uint(FixedSizeIndex::START) << std::endl;
		std::cout << "  HEADER: " << to_uint(FixedSizeIndex::HEADER) << std::endl;
		std::cout << "  TYPE: " << to_uint(FixedSizeIndex::TYPE) << std::endl;
		std::cout << "  ID_0: " << to_uint(FixedSizeIndex::ID_0) << std::endl;
		std::cout << "  DLC: " << to_uint(FixedSizeIndex::DLC) << std::endl;
		std::cout << "  DATA_0: " << to_uint(FixedSizeIndex::DATA_0) << std::endl;
		std::cout << "  CHECKSUM: " << to_uint(FixedSizeIndex::CHECKSUM) << std::endl;

		std::cout << "\nVariable Frame Indices:" << std::endl;
		std::cout << "  START: " << to_uint(VarSizeIndex::START) << std::endl;
		std::cout << "  TYPE_HEADER: " << to_uint(VarSizeIndex::TYPE_HEADER) << std::endl;
		std::cout << "  ID_START: " << to_uint(VarSizeIndex::ID_START) << std::endl;

		// Test VarSizeIndex arithmetic
		VarSizeIndex test_idx = VarSizeIndex::ID_START + 2;
		std::cout << "  ID_START + 2: " << to_uint(test_idx) << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}

	std::cout << "\n=== Demo completed successfully ===" << std::endl;
	return 0;
}
