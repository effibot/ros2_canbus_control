/**
 * @file usb_can_usage_example.cpp
 * @brief Example demonstrating the usage of the refactored USB-CAN frame classes
 * This file shows how to use the new namespace structure and utility functions
 * to avoid repetitive static_cast operations.
 */

#include "usb_can_frame.hpp"
#include <iostream>
#include <iomanip>

using namespace USBCANBridge;

void demonstrateUtilityFunctions() {
	std::cout << "=== Demonstrating Utility Functions ===\n";

	// Before refactoring (verbose static_cast):
	// uint8_t start_byte = static_cast<uint8_t>(USBCANConst::START_BYTE);

	// After refactoring (clean utility functions):
	uint8_t start_byte = to_uint8(USBCANConst::START_BYTE);
	uint8_t msg_header = to_uint8(USBCANConst::MSG_HEADER);
	uint8_t end_byte = to_uint8(USBCANConst::END_BYTE);

	std::cout << "Start byte: 0x" << std::hex << static_cast<int>(start_byte) << std::dec << "\n";
	std::cout << "Message header: 0x" << std::hex << static_cast<int>(msg_header) << std::dec << "\n";
	std::cout << "End byte: 0x" << std::hex << static_cast<int>(end_byte) << std::dec << "\n";

	// Frame type conversions
	uint8_t std_fixed = to_uint8(USBCANFrameType::STD_FIXED);
	uint8_t ext_fixed = to_uint8(USBCANFrameType::EXT_FIXED);

	std::cout << "Standard fixed frame type: 0x" << std::hex << static_cast<int>(std_fixed) << std::dec << "\n";
	std::cout << "Extended fixed frame type: 0x" << std::hex << static_cast<int>(ext_fixed) << std::dec << "\n";

	// Baud rate conversions
	uint8_t can_speed = to_uint8(USBCANBaud::SPEED_1000K);
	uint32_t usb_baud = to_uint32(USBBaud::BAUD_115200);

	std::cout << "CAN speed 1000K: 0x" << std::hex << static_cast<int>(can_speed) << std::dec << "\n";
	std::cout << "USB baud 115200: " << usb_baud << "\n";
}

void demonstrateDefaultConstants() {
	std::cout << "\n=== Demonstrating Default Constants ===\n";

	// Using the new constexpr constants instead of #defines
	std::cout << "Default CAN speed: 0x" << std::hex << static_cast<int>(to_uint8(USB_DEF_CAN_SPEED)) << std::dec << "\n";
	std::cout << "Default baud rate: " << to_uint32(USB_DEF_BAUD_RATE) << "\n";
	std::cout << "Default CAN mode: 0x" << std::hex << static_cast<int>(to_uint8(USB_DEF_CAN_MODE)) << std::dec << "\n";
	std::cout << "Default RTX setting: 0x" << std::hex << static_cast<int>(to_uint8(USB_DEF_RTX)) << std::dec << "\n";
}

void demonstrateVariableFrame() {
	std::cout << "\n=== Demonstrating Variable Frame ===\n";

	// Create a variable frame using the new namespace
	USBCANAdapterVariableFrame frame(
		to_uint8(USBCANFrameFmt::STD), // Standard frame
		USBCANFrameFmtVar::DATA,   // Data frame (not remote)
		8                          // DLC = 8 bytes
		);

	// Set ID and data
	std::vector<uint8_t> id_bytes = {0x23, 0x01}; // Standard ID: 0x0123
	std::vector<uint8_t> data_bytes = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

	frame.setIDBytes(id_bytes);
	frame.setDataBytes(data_bytes);

	std::cout << "Frame size: " << frame.size() << " bytes\n";
	std::cout << "Valid start: " << (frame.isValidStart() ? "Yes" : "No") << "\n";
	std::cout << "Valid end: " << (frame.isValidEnd() ? "Yes" : "No") << "\n";
	std::cout << "Valid length: " << (frame.isValidLength() ? "Yes" : "No") << "\n";
}

void demonstrateFrameTypeComparison() {
	std::cout << "\n=== Demonstrating Frame Type Comparison ===\n";

	// Easy comparison without static_cast everywhere
	USBCANFrameType current_type = USBCANFrameType::STD_FIXED;

	if (current_type == USBCANFrameType::STD_FIXED) {
		std::cout << "Frame is standard fixed type\n";
	} else if (current_type == USBCANFrameType::EXT_FIXED) {
		std::cout << "Frame is extended fixed type\n";
	}

	// When you need the raw byte value:
	uint8_t type_byte = to_uint8(current_type);
	std::cout << "Frame type as byte: 0x" << std::hex << static_cast<int>(type_byte) << std::dec << "\n";
}

int main() {
	std::cout << "USB-CAN Bridge Refactoring Demonstration\n";
	std::cout << "========================================\n";

	demonstrateUtilityFunctions();
	demonstrateDefaultConstants();
	demonstrateVariableFrame();
	demonstrateFrameTypeComparison();

	std::cout << "\n=== Benefits of Refactoring ===\n";
	std::cout << "1. All enums and constants are now in USBCANBridge namespace\n";
	std::cout << "2. Utility functions eliminate repetitive static_cast operations\n";
	std::cout << "3. Constexpr constants replace #define macros\n";
	std::cout << "4. Type safety is maintained with strongly-typed enums\n";
	std::cout << "5. Code is more readable and maintainable\n";

	return 0;
}
