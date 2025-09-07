#include "usb_can_frames.hpp"
#include <iostream>
#include <iomanip>

using namespace USBCANBridge;
void printFrameData(const std::vector<uint8_t>& data) {
	std::cout << "Frame data: ";
	for (const auto& byte : data) {
		std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
		          << static_cast<int>(byte) << " ";
	}
	std::cout << std::dec << std::endl;
}


std::vector<uint8_t> buildRawFixedFrame() {
	// Example fixed frame: Standard ID, DATA frame, ID=0x123, DLC=8, Data=0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88
	return {
	        0xAA,           // Start byte
	        0x55,           // Message header
	        0x01,           // Type: DATA
	        0x01,           // Frame type: STD_FIXED
	        0x01,           // Frame format: DATA_FIXED
	        0x23, 0x01, 0x00, 0x00, // ID: 0x00000123 (little-endian)
	        0x08,           // DLC: 8
	        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // Data bytes
	        0x00,           // Reserved
	        0x93            // Checksum (example value)
	};
}
std::vector<uint8_t> buildRawFixedFrame_ext(){
	// Example fixed frame: Extended ID, DATA frame, ID=0x12345678, Data=01 02 03 04 05 06 07 08
	return {
	        0xAA,       // Start byte
	        0x55,       // Message header
	        0x01,       // Type: DATA
	        0x02,       // Frame type: EXT_FIXED
	        0x01,       // Frame format: DATA_FIXED
	        0x78, 0x56, 0x34, 0x12, // ID: 0x12345678 (little-endian)
	        0x08,       // DLC: 8
	        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // Data bytes
	        0x00,       // Reserved
	        0x44        // Checksum (example value)
	};
}

void testStandardFixedFrame() {
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
		std::vector<uint8_t> test_data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
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
		printFrameData(buildRawFixedFrame());

		// Demonstrate individual byte access
		std::cout << "Frame type: 0x" << std::hex << static_cast<int>((*frame)[to_uint(FixedSizeIndex::FRAME_TYPE)]) << std::dec << std::endl;
		std::cout << "DLC: " << static_cast<int>((*frame)[to_uint(FixedSizeIndex::DLC)]) << std::endl;

	} catch (const std::exception& e) {
		std::cerr << "Error with fixed frame: " << e.what() << std::endl;
	}
}

void testExtendedFixedFrame() {
	std::cout << "\n=== Extended Fixed Frame Example ===" << std::endl;

	auto frame_ext = FrameFactory::createFixedFrame(
		Type::DATA_FIXED,
		FrameType::EXT_FIXED,
		FrameFmt::DATA_FIXED
		);
	frame_ext->setID(0x12345678);
	std::vector<uint8_t> test_data_ext = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	frame_ext->setData(test_data_ext);
	frame_ext->setDLC(test_data_ext.size());

	// frame_ext->updateChecksum();
	if (frame_ext->isValidFrame()) {
		std::cout << "Extended Frame is valid!" << std::endl;
	} else {
		std::cout << "Extended Frame is invalid!" << std::endl;
	}
	auto serialized_ext = frame_ext->serialize();
	std::cout << "Serialized extended frame (" << serialized_ext.size() << " bytes):" << std::endl;
	printFrameData(serialized_ext);
	printFrameData(buildRawFixedFrame_ext());
}

std::vector<uint8_t> buildRawVarFrame() {
	// Example variable frame: Standard ID, DATA frame, ID=0x123, DLC=8, Data= 0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88
	return {
	        0xAA,       // Start byte
	        0xC8,       // Type: C0 + frame type + frame format + DLC
	        0x23, 0x01, // ID: 0x0123 (little-endian)
	        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // Data bytes
	        0x55        // End byte
	};
}

std::vector<uint8_t> buildRawVarFrame_ext() {
	// Example variable frame: Extended ID, DATA frame, ID=0x1234567, DLC=8, Data= 0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88
	return {
	        0xAA,   // Start byte
	        0xE8,   // Type: C0 + frame type + frame format + DLC
	        0x67, 0x45, 0x23, 01, // ID: 0x12345678 (little-endian)
	        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // Data bytes
	        0x55    // End byte
	};
}

void testStandardVarFrame() {
	std::cout << "\n=== Variable Frame Example ===" << std::endl;

	try {
		// Create a standard variable frame using the factory
		auto frame = FrameFactory::createVariableFrame(
			FrameType::STD_VAR,
			FrameFmt::DATA_VAR
			);

		std::cout << "Created variable frame with size: " << frame->size() << " bytes" << std::endl;

		// Set CAN ID
		frame->setID(0x123);
		std::cout << "Set CAN ID: 0x" << std::hex << frame->getID() << std::dec << std::endl;

		// Set data
		std::vector<uint8_t> test_data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
		frame->setData(test_data);
		frame->setDLC(test_data.size());

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
		printFrameData(buildRawVarFrame());

	} catch (const std::exception& e) {
		std::cerr << "Error with variable frame: " << e.what() << std::endl;
	}
}

void testExtendedVarFrame() {
	std::cout << "\n=== Extended Variable Frame Example ===" << std::endl;

	auto frame_ext = FrameFactory::createVariableFrame(
		FrameType::EXT_VAR,
		FrameFmt::DATA_VAR
		);
	frame_ext->setID(0x1234567);
	std::vector<uint8_t> test_data_ext = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	frame_ext->setData(test_data_ext);
	frame_ext->setDLC(test_data_ext.size());

	std::cout << "Set CAN ID: 0x" << std::hex << frame_ext->getID() << std::dec << std::endl;
	std::cout << "Set data length: " << static_cast<int>(frame_ext->getDLC()) << static_cast<uint8_t>(frame_ext->getDLC()) << std::endl;

	if (frame_ext->isValidFrame()) {
		std::cout << "Extended Variable Frame is valid!" << std::endl;
	} else {
		std::cout << "Extended Variable Frame is invalid!" << std::endl;
	}
	auto serialized_ext = frame_ext->serialize();
	std::cout << "Serialized extended variable frame (" << serialized_ext.size() << " bytes):" << std::endl;
	printFrameData(serialized_ext);
	printFrameData(buildRawVarFrame_ext());

	uint8_t type_byte = 0xC0 + 1 + 0 + static_cast<uint8_t>(frame_ext->getDLC());
	std::cout << "Constructed type byte: 0x" << std::hex << static_cast<int>(type_byte) << std::dec << std::endl;
}

int main() {
	// testStandardFixedFrame();
	// testExtendedFixedFrame();
	// testStandardVarFrame();
	testExtendedVarFrame();
	return 0;
}