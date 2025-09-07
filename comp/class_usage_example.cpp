/**
 * @file class_usage_example.cpp
 * @brief Example demonstrating the usage of the class-based USB-CAN frame implementation
 * This example shows how to create, manipulate, and serialize/deserialize USB-CAN frames
 * using the class-based approach instead of structs.
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#include "usb_can_frames.hpp"
#include <iostream>
#include <iomanip>
#include <catch2/catch_test_macros.hpp>

using namespace USBCANBridge;

void printFrameData(const std::vector<uint8_t>& data) {
	std::cout << "Frame data: ";
	for (const auto& byte : data) {
		std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
		          << static_cast<int>(byte) << " ";
	}
	std::cout << std::dec << std::endl;
}

void demonstrateFixedFrame() {
	std::cout << "\n=== Fixed Frame Example ===" << std::endl;

	try {
		// Create a standard fixed frame using the factory
		auto frame = FrameFactory::createFixedFrame(
			Type::DATA_FIXED,
			FrameType::STD_FIXED,
			FrameFmt::DATA_FIXED
			);

		std::cout << "Created fixed frame with size: " << frame->size() << " bytes" << std::endl;

		// Set CAN ID
		frame->setID(0x123);
		std::cout << "Set CAN ID: 0x" << std::hex << frame->getID() << std::dec << std::endl;

		// Set data
		std::vector<uint8_t> test_data = {0x01, 0x02, 0x03, 0x04};
		frame->setData(test_data);
		frame->setDLC(test_data.size());

		std::cout << "Set data length: " << static_cast<int>(frame->getDLC()) << std::endl;

		// Update checksum (automatic in class version)
		frame->updateChecksum();

		// Validate frame
		if (frame->isValidFrame()) {
			std::cout << "Frame is valid!" << std::endl;
		} else {
			std::cout << "Frame is invalid!" << std::endl;
		}

		// Serialize frame
		auto serialized = frame->serialize();
		std::cout << "Serialized frame (" << serialized.size() << " bytes):" << std::endl;
		printFrameData(serialized);

		// Demonstrate individual byte access
		std::cout << "Frame type: 0x" << std::hex << static_cast<int>((*frame)[to_uint(FixedSizeIndex::FRAME_TYPE)]) << std::dec << std::endl;
		std::cout << "DLC: " << static_cast<int>((*frame)[to_uint(FixedSizeIndex::DLC)]) << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Error with fixed frame: " << e.what() << std::endl;
	}
}

void demonstrateVariableFrame() {
	std::cout << "\n=== Variable Frame Example ===" << std::endl;

	try {
		// Create an extended variable frame using the factory
		auto frame = FrameFactory::createVariableFrame(
			FrameType::EXT_VAR,
			FrameFmt::DATA_VAR,
			6 // DLC = 6 bytes
			);

		std::cout << "Created variable frame with size: " << frame->size() << " bytes" << std::endl;

		// Set extended CAN ID
		frame->setID(0x1ABCDEF);
		std::cout << "Set extended CAN ID: 0x" << std::hex << frame->getID() << std::dec << std::endl;

		// Set data
		std::vector<uint8_t> test_data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
		frame->setData(test_data);

		std::cout << "Set data length: " << static_cast<int>(frame->getDLC()) << std::endl;

		// Validate frame
		if (frame->isValidFrame()) {
			std::cout << "Frame is valid!" << std::endl;
		} else {
			std::cout << "Frame is invalid!" << std::endl;
		}

		// Serialize frame
		auto serialized = frame->serialize();
		std::cout << "Serialized frame (" << serialized.size() << " bytes):" << std::endl;
		printFrameData(serialized);

		// Get frame properties
		std::cout << "Frame type: " << (frame->getFrameType() == FrameType::EXT_VAR ? "Extended" : "Standard") << std::endl;
		std::cout << "Frame format: " << (frame->getFrameFmt() == FrameFmt::DATA_VAR ? "Data" : "Remote") << std::endl;

		// Demonstrate ID bytes access
		auto id_bytes = frame->getIDBytes();
		std::cout << "ID bytes: ";
		for (const auto& byte : id_bytes) {
			std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
			          << static_cast<int>(byte) << " ";
		}
		std::cout << std::dec << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Error with variable frame: " << e.what() << std::endl;
	}
}

void demonstrateDeserialization() {
	std::cout << "\n=== Deserialization Example ===" << std::endl;

	try {
		// Simulate raw frame data (variable frame with standard ID)
		std::vector<uint8_t> raw_data = {
			0xAA, // Start byte
			0xC6, // Type byte (0xC0 | STD_VAR | DATA_VAR | DLC=6)
			0x34, 0x12, // ID bytes (little-endian: 0x1234)
			0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Data bytes
			0x55 // End byte
		};

		std::cout << "Raw frame data:" << std::endl;
		printFrameData(raw_data);

		// Auto-detect and create frame
		auto frame = FrameFactory::createFrameFromData(raw_data);

		std::cout << "Successfully parsed frame!" << std::endl;
		std::cout << "Frame size: " << frame->size() << " bytes" << std::endl;
		std::cout << "CAN ID: 0x" << std::hex << frame->getID() << std::dec << std::endl;
		std::cout << "DLC: " << static_cast<int>(frame->getDLC()) << std::endl;

		// Get and display data
		auto data = frame->getData();
		std::cout << "Frame data: ";
		for (const auto& byte : data) {
			std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
			          << static_cast<int>(byte) << " ";
		}
		std::cout << std::dec << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Error with deserialization: " << e.what() << std::endl;
	}
}

void demonstratePolymorphism() {
	std::cout << "\n=== Polymorphism Example ===" << std::endl;

	try {
		// Create different frame types and store them in a common container
		std::vector<std::unique_ptr<AdapterBaseFrame> > frames;

		// Add a fixed frame
		frames.push_back(FrameFactory::createFixedFrame(
					 Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED));

		// Add a variable frame
		frames.push_back(FrameFactory::createVariableFrame(
					 FrameType::STD_VAR, FrameFmt::DATA_VAR, 4));

		// Process all frames polymorphically
		for (size_t i = 0; i < frames.size(); ++i) {
			auto& frame = frames[i];

			std::cout << "Frame " << i + 1 << ":" << std::endl;
			std::cout << "  Size: " << frame->size() << " bytes" << std::endl;
			std::cout << "  Type: " << (frame->getType() == Type::DATA_FIXED ? "Fixed" : "Variable") << std::endl;

			// Set some test data
			frame->setID(0x100 + i);
			std::vector<uint8_t> test_data = {0x10, 0x20, 0x30, 0x40};
			frame->setData(test_data);
			frame->setDLC(test_data.size());

			std::cout << "  CAN ID: 0x" << std::hex << frame->getID() << std::dec << std::endl;
			std::cout << "  Valid: " << (frame->isValidFrame() ? "Yes" : "No") << std::endl;

			// Serialize and show size
			try {
				auto serialized = frame->serialize();
				std::cout << "  Serialized size: " << serialized.size() << " bytes" << std::endl;
			} catch (const std::exception& e) {
				std::cout << "  Serialization failed: " << e.what() << std::endl;
			}
		}

	} catch (const std::exception& e) {
		std::cerr << "Error with polymorphism example: " << e.what() << std::endl;
	}
}

void demonstrateErrorHandling() {
	std::cout << "\n=== Error Handling Example ===" << std::endl;

	try {
		auto frame = FrameFactory::createFixedFrame(
			Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);

		// Try to set invalid DLC
		try {
			frame->setDLC(15); // Invalid: max is 8
		} catch (const std::invalid_argument& e) {
			std::cout << "Caught expected error for invalid DLC: " << e.what() << std::endl;
		}

		// Try to set invalid standard ID
		try {
			frame->setID(0x1000); // Invalid: max for standard is 0x7FF
		} catch (const std::invalid_argument& e) {
			std::cout << "Caught expected error for invalid ID: " << e.what() << std::endl;
		}

		// Try to access invalid data index
		try {
			frame->setData(10, 0xFF); // Invalid: max index is 7
		} catch (const std::out_of_range& e) {
			std::cout << "Caught expected error for invalid index: " << e.what() << std::endl;
		}

		std::cout << "Error handling working correctly!" << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Unexpected error: " << e.what() << std::endl;
	}
}

int main() {
	std::cout << "USB-CAN Frame Class-Based Implementation Example" << std::endl;
	std::cout << "================================================" << std::endl;

	demonstrateFixedFrame();
	demonstrateVariableFrame();
	demonstrateDeserialization();
	demonstratePolymorphism();
	demonstrateErrorHandling();

	std::cout << "\nExample completed successfully!" << std::endl;
	return 0;
}
