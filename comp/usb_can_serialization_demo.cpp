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

// Function to manually create a serialized-like representation
std::vector<uint8_t> manualFixedFrameSerial(const fixedSize& frame) {
	std::vector<uint8_t> result;
	result.reserve(20);

	// Add each byte manually using the [] operator
	for (size_t i = 0; i < frame.size(); ++i) {
		result.push_back(frame[i]);
	}

	return result;
}

// Function to manually create a variable frame serialized representation
std::vector<uint8_t> manualVariableFrameSerial(const varSize& frame) {
	std::vector<uint8_t> result;

	// Start byte
	result.push_back(frame[0]);

	// Type byte (constructed manually)
	uint8_t type_byte = make_variable_type_byte(frame.getFrameType(), frame.getFrameFmt(), frame.getDLC());
	result.push_back(type_byte);

	// ID bytes
	auto id_bytes = frame.getIDBytes();
	result.insert(result.end(), id_bytes.begin(), id_bytes.end());

	// Data bytes
	auto data_bytes = frame.getData();
	result.insert(result.end(), data_bytes.begin(), data_bytes.end());

	// End byte
	result.push_back(to_uint8(Constants::END_BYTE));

	return result;
}

int main() {
	std::cout << "=== USB-CAN Frame Serialization Demo ===" << std::endl;

	try {
		// ========== Fixed Size Frame Build and Serialize ==========
		std::cout << "\n--- Building Fixed Size Frame ---" << std::endl;

		// Create a 20-byte fixed frame for standard data frame
		fixedSize fixed_frame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);

		// Configure the frame
		uint32_t can_id = 0x123; // Standard CAN ID
		fixed_frame.setID(can_id);
		fixed_frame.setDLC(8); // 8 bytes of data

		// Set sample data byte by byte
		for (size_t i = 0; i < 8; ++i) {
			fixed_frame.setData(i, static_cast<uint8_t>(0x10 + i));
		}

		std::cout << "Fixed Frame Configuration:" << std::endl;
		std::cout << "  CAN ID: 0x" << std::hex << fixed_frame.getID() << std::dec << std::endl;
		std::cout << "  DLC: " << static_cast<int>(fixed_frame.getDLC()) << std::endl;
		std::cout << "  Type: " << static_cast<int>(to_uint8(fixed_frame.getType())) << std::endl;
		std::cout << "  Frame Type: " << static_cast<int>(to_uint8(fixed_frame.getFrameType())) << std::endl;
		std::cout << "  Frame Format: " << static_cast<int>(to_uint8(fixed_frame.getFrameFmt())) << std::endl;

		// Attempt manual serialization
		auto fixed_serialized = manualFixedFrameSerial(fixed_frame);
		printBytes(fixed_serialized, "Fixed Frame Serialized");

		// ========== Variable Size Frame Build and Serialize ==========
		std::cout << "\n--- Building Variable Size Frame ---" << std::endl;

		// Create a variable length frame for extended data frame
		varSize var_frame(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 4);

		// Configure the frame
		uint32_t extended_can_id = 0x12345678; // Extended CAN ID
		var_frame.setID(extended_can_id);

		// Set sample data
		for (size_t i = 0; i < 4; ++i) {
			var_frame.setData(i, static_cast<uint8_t>(0xA0 + i));
		}

		std::cout << "Variable Frame Configuration:" << std::endl;
		std::cout << "  CAN ID: 0x" << std::hex << var_frame.getID() << std::dec << std::endl;
		std::cout << "  DLC: " << static_cast<int>(var_frame.getDLC()) << std::endl;
		std::cout << "  Frame Type: " << (var_frame.getFrameType() == FrameType::EXT_VAR ? "Extended" : "Standard") << std::endl;
		std::cout << "  Frame Format: " << (var_frame.getFrameFmt() == FrameFmt::DATA_VAR ? "Data" : "Remote") << std::endl;
		std::cout << "  Frame Size: " << var_frame.size() << " bytes" << std::endl;

		// Attempt manual serialization
		auto var_serialized = manualVariableFrameSerial(var_frame);
		printBytes(var_serialized, "Variable Frame Serialized");

		// ========== Standard vs Extended Comparison ==========
		std::cout << "\n--- Standard vs Extended Frame Comparison ---" << std::endl;

		// Standard frame
		varSize std_frame(FrameType::STD_VAR, FrameFmt::DATA_VAR, 3);
		std_frame.setID(0x7FF); // Max standard ID
		for (size_t i = 0; i < 3; ++i) {
			std_frame.setData(i, static_cast<uint8_t>(0x01 + i));
		}

		auto std_serialized = manualVariableFrameSerial(std_frame);
		printBytes(std_serialized, "Standard ID Frame (11-bit ID)");

		// Extended frame
		varSize ext_frame(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 3);
		ext_frame.setID(0x1FFFFFFF); // Max extended ID
		for (size_t i = 0; i < 3; ++i) {
			ext_frame.setData(i, static_cast<uint8_t>(0x04 + i));
		}

		auto ext_serialized = manualVariableFrameSerial(ext_frame);
		printBytes(ext_serialized, "Extended ID Frame (29-bit ID)");

		// ========== Remote Frame Example ==========
		std::cout << "\n--- Remote Frame Example ---" << std::endl;

		// Create a remote frame (no data, just request)
		varSize remote_frame(FrameType::STD_VAR, FrameFmt::REMOTE_VAR, 0);
		remote_frame.setID(0x456); // Request data from this ID

		auto remote_serialized = manualVariableFrameSerial(remote_frame);
		printBytes(remote_serialized, "Remote Frame (no data)");

		std::cout << "Remote Frame Properties:" << std::endl;
		std::cout << "  CAN ID: 0x" << std::hex << remote_frame.getID() << std::dec << std::endl;
		std::cout << "  DLC: " << static_cast<int>(remote_frame.getDLC()) << " (no data expected)" << std::endl;

		// ========== Frame Analysis ==========
		std::cout << "\n--- Frame Analysis ---" << std::endl;

		std::cout << "Frame Size Comparison:" << std::endl;
		std::cout << "  Fixed Frame: " << fixed_serialized.size() << " bytes (always 20)" << std::endl;
		std::cout << "  Standard Variable: " << std_serialized.size() << " bytes" << std::endl;
		std::cout << "  Extended Variable: " << ext_serialized.size() << " bytes" << std::endl;
		std::cout << "  Remote Variable: " << remote_serialized.size() << " bytes" << std::endl;

		std::cout << "\nType Byte Analysis:" << std::endl;
		std::cout << "  Fixed frame type byte: 0x" << std::hex << static_cast<int>(fixed_serialized[2]) << std::dec << std::endl;
		std::cout << "  Standard var type byte: 0x" << std::hex << static_cast<int>(std_serialized[1]) << std::dec << std::endl;
		std::cout << "  Extended var type byte: 0x" << std::hex << static_cast<int>(ext_serialized[1]) << std::dec << std::endl;
		std::cout << "  Remote var type byte: 0x" << std::hex << static_cast<int>(remote_serialized[1]) << std::dec << std::endl;

		// ========== Protocol Testing ==========
		std::cout << "\n--- Protocol Testing ---" << std::endl;

		// Test make_variable_type_byte function
		std::cout << "Variable Type Byte Generation:" << std::endl;

		uint8_t test_type1 = make_variable_type_byte(FrameType::STD_VAR, FrameFmt::DATA_VAR, 8);
		std::cout << "  STD + DATA + DLC=8: 0x" << std::hex << static_cast<int>(test_type1) << std::dec << std::endl;

		uint8_t test_type2 = make_variable_type_byte(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 8);
		std::cout << "  EXT + DATA + DLC=8: 0x" << std::hex << static_cast<int>(test_type2) << std::dec << std::endl;

		uint8_t test_type3 = make_variable_type_byte(FrameType::STD_VAR, FrameFmt::REMOTE_VAR, 0);
		std::cout << "  STD + REMOTE + DLC=0: 0x" << std::hex << static_cast<int>(test_type3) << std::dec << std::endl;

		uint8_t test_type4 = make_variable_type_byte(FrameType::EXT_VAR, FrameFmt::REMOTE_VAR, 0);
		std::cout << "  EXT + REMOTE + DLC=0: 0x" << std::hex << static_cast<int>(test_type4) << std::dec << std::endl;

		// Test parse_variable_type_byte function
		std::cout << "\nVariable Type Byte Parsing:" << std::endl;
		FrameType parsed_type;
		FrameFmt parsed_fmt;
		uint8_t parsed_dlc;

		parse_variable_type_byte(test_type1, parsed_type, parsed_fmt, parsed_dlc);
		std::cout << "  Parsed 0x" << std::hex << static_cast<int>(test_type1) << std::dec << ": "
		          << (parsed_type == FrameType::STD_VAR ? "STD" : "EXT") << " + "
		          << (parsed_fmt == FrameFmt::DATA_VAR ? "DATA" : "REMOTE") << " + "
		          << "DLC=" << static_cast<int>(parsed_dlc) << std::endl;

		parse_variable_type_byte(test_type2, parsed_type, parsed_fmt, parsed_dlc);
		std::cout << "  Parsed 0x" << std::hex << static_cast<int>(test_type2) << std::dec << ": "
		          << (parsed_type == FrameType::STD_VAR ? "STD" : "EXT") << " + "
		          << (parsed_fmt == FrameFmt::DATA_VAR ? "DATA" : "REMOTE") << " + "
		          << "DLC=" << static_cast<int>(parsed_dlc) << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}

	std::cout << "\n=== Serialization Demo completed successfully ===" << std::endl;
	return 0;
}
