#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "fixed_size_frame.hpp"
#include "variable_size_frame.hpp"
#include "frame_factory.hpp"
#include "usb_can_common.hpp"

using namespace USBCANBridge;

// Helper to build expected fixed frame bytes (20 bytes)
static std::vector<uint8_t> build_expected_fixed(Type type,
                                                 FrameType frame_type,
                                                 FrameFmt frame_fmt,
                                                 uint32_t id,
                                                 uint8_t dlc,
                                                 const std::vector<uint8_t>& data) {
	REQUIRE(dlc <= 8);
	REQUIRE(data.size() <= 8);
	std::vector<uint8_t> bytes;
	bytes.reserve(20);
	bytes.push_back(to_uint8(Constants::START_BYTE)); // 0
	bytes.push_back(to_uint8(Constants::MSG_HEADER)); // 1
	bytes.push_back(to_uint8(type));             // 2
	bytes.push_back(to_uint8(frame_type));       // 3
	bytes.push_back(to_uint8(frame_fmt));        // 4
	// ID little endian into 4 bytes
	bytes.push_back(static_cast<uint8_t>(id & 0xFF));   // 5
	bytes.push_back(static_cast<uint8_t>((id >> 8) & 0xFF));// 6
	bytes.push_back(static_cast<uint8_t>((id >> 16) & 0xFF));//7
	bytes.push_back(static_cast<uint8_t>((id >> 24) & 0xFF));//8
	bytes.push_back(dlc);                              //9
	for (size_t i=0; i<8; i++) {
		if (i < data.size()) bytes.push_back(data[i]); else bytes.push_back(0x00); // 10-17
	}
	bytes.push_back(to_uint8(Constants::RESERVED0));    // 18
	// checksum: low 8 bits of sum from type_ to reserved_
	uint32_t sum = 0;
	sum += to_uint8(type);
	sum += to_uint8(frame_type);
	sum += to_uint8(frame_fmt);
	sum += bytes[5] + bytes[6] + bytes[7] + bytes[8];
	sum += dlc;
	for (size_t i=10; i<18; i++) sum += bytes[i];
	sum += bytes[18];
	bytes.push_back(static_cast<uint8_t>(sum & 0xFF)); // 19
	REQUIRE(bytes.size()==20);
	return bytes;
}

TEST_CASE("FixedSizeFrame serialization standard id", "[fixed][serialize]") {
	auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
	frame->setID(123);
	std::vector<uint8_t> payload = {0x11,0x22,0x33,0x44};
	frame->setData(payload);
	frame->setDLC(static_cast<uint8_t>(payload.size()));
	frame->updateChecksum();

	auto serialized = frame->serialize();
	auto expected = build_expected_fixed(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED, 123, (uint8_t)payload.size(), payload);
	REQUIRE(serialized == expected);
	REQUIRE(frame->isValidFrame());
}

TEST_CASE("FixedSizeFrame serialize extended id", "[fixed][serialize]") {
	auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::EXT_FIXED, FrameFmt::DATA_FIXED);
	frame->setID(0x1ABCDEF); // within 29-bit range
	std::vector<uint8_t> payload = {0xDE,0xAD,0xBE,0xEF,0x01};
	frame->setData(payload);
	frame->setDLC(static_cast<uint8_t>(payload.size()));
	frame->updateChecksum();
	auto serialized = frame->serialize();
	auto expected = build_expected_fixed(Type::DATA_FIXED, FrameType::EXT_FIXED, FrameFmt::DATA_FIXED, 0x1ABCDEF, (uint8_t)payload.size(), payload);
	REQUIRE(serialized == expected);
}

TEST_CASE("FixedSizeFrame invalid standard id rejected", "[fixed][error]") {
	auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
	REQUIRE_THROWS_AS(frame->setID(0x800), std::invalid_argument); // 0x7FF max
}

TEST_CASE("FixedSizeFrame invalid extended id rejected", "[fixed][error]") {
	auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::EXT_FIXED, FrameFmt::DATA_FIXED);
	REQUIRE_THROWS_AS(frame->setID(0x20000000), std::invalid_argument); // > 29 bits
}

TEST_CASE("FixedSizeFrame invalid DLC rejected", "[fixed][error]") {
	auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
	REQUIRE_THROWS_AS(frame->setDLC(9), std::invalid_argument);
}

TEST_CASE("FixedSizeFrame deserialize roundtrip", "[fixed][deserialize]") {
	// Build a frame via API then deserialize the serialized bytes
	auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::EXT_FIXED, FrameFmt::DATA_FIXED);
	std::vector<uint8_t> payload = {0xAA,0xBB,0xCC,0xDD,0xEE};
	frame->setID(0x1ABCDE0);
	frame->setData(payload);
	frame->setDLC((uint8_t)payload.size());
	frame->updateChecksum();
	auto serialized = frame->serialize();

	auto parsed = FrameFactory::createFrameFromData(serialized);
	REQUIRE(parsed->getFrameType() == FrameType::EXT_FIXED);
	REQUIRE(parsed->getType() == Type::DATA_FIXED);
	REQUIRE(parsed->getDLC() == payload.size());
	REQUIRE(parsed->getID() == 0x1ABCDE0);
	for (size_t i=0; i<payload.size(); ++i) REQUIRE(parsed->getData()[i]==payload[i]);
	REQUIRE(parsed->serialize() == serialized);
}

// ---------------- Variable frame tests -----------------

TEST_CASE("VariableSizeFrame serialization standard", "[var][serialize]") {
	auto frame = FrameFactory::createVariableFrame(FrameType::STD_VAR, FrameFmt::DATA_VAR, 3);
	frame->setID(0x456);
	std::vector<uint8_t> data = {0x10,0x20,0x30};
	frame->setData(data);
	REQUIRE(frame->getDLC()==3);
	auto serialized = frame->serialize();
	// Build expected: start, type byte, id(2), data(3), end
	uint8_t type_byte = make_variable_type_byte(FrameType::STD_VAR, FrameFmt::DATA_VAR, 3);
	std::vector<uint8_t> expected = {to_uint8(Constants::START_BYTE), type_byte, 0x56, 0x04, 0x10,0x20,0x30, to_uint8(Constants::END_BYTE)}; // ID little endian 0x0456
	REQUIRE(serialized == expected);
}

TEST_CASE("VariableSizeFrame serialization extended", "[var][serialize]") {
	auto frame = FrameFactory::createVariableFrame(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 0);
	frame->setID(0x1ABCDE0);
	frame->setData({});
	uint8_t type_byte = make_variable_type_byte(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 0);
	auto serialized = frame->serialize();
	std::vector<uint8_t> expected = {to_uint8(Constants::START_BYTE), type_byte,
		                         0xE0, 0xCD, 0xAB, 0x01,
		                         to_uint8(Constants::END_BYTE)};
	REQUIRE(serialized == expected);
}

TEST_CASE("VariableSizeFrame invalid standard id rejected", "[var][error]") {
	auto frame = FrameFactory::createVariableFrame(FrameType::STD_VAR, FrameFmt::DATA_VAR, 0);
	REQUIRE_THROWS_AS(frame->setID(0x800), std::invalid_argument);
}

TEST_CASE("VariableSizeFrame invalid extended id rejected", "[var][error]") {
	auto frame = FrameFactory::createVariableFrame(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 0);
	REQUIRE_THROWS_AS(frame->setID(0x20000000), std::invalid_argument);
}

TEST_CASE("VariableSizeFrame invalid DLC rejected", "[var][error]") {
	auto frame = FrameFactory::createVariableFrame(FrameType::STD_VAR, FrameFmt::DATA_VAR, 0);
	REQUIRE_THROWS_AS(frame->setDLC(9), std::invalid_argument);
}

TEST_CASE("VariableSizeFrame deserialize roundtrip", "[var][deserialize]") {
	// Build raw frame: start, type byte (STD, DATA, DLC=4) id(0x0456 little endian) data(4) end
	uint8_t type_byte = make_variable_type_byte(FrameType::STD_VAR, FrameFmt::DATA_VAR, 4);
	std::vector<uint8_t> raw = {to_uint8(Constants::START_BYTE), type_byte, 0x56,0x04, 0x01,0x02,0x03,0x04, to_uint8(Constants::END_BYTE)};
	auto frame = FrameFactory::createFrameFromData(raw);
	REQUIRE(frame->getFrameType()==FrameType::STD_VAR);
	REQUIRE(frame->getID()==0x456);
	REQUIRE(frame->getDLC()==4);
	auto data = frame->getData();
	REQUIRE(data.size()==4);
	REQUIRE(data[0]==0x01);
	REQUIRE(data[3]==0x04);
	auto reserialized = frame->serialize();
	REQUIRE(reserialized == raw);
}

TEST_CASE("VariableSizeFrame deserialize invalid start byte", "[var][deserialize][error]") {
	uint8_t type_byte = make_variable_type_byte(FrameType::STD_VAR, FrameFmt::DATA_VAR, 1);
	std::vector<uint8_t> raw = {0xAB, type_byte, 0x01,0x00, to_uint8(Constants::END_BYTE)}; // wrong start byte
	REQUIRE_THROWS_AS(FrameFactory::createFrameFromData(raw), std::invalid_argument);
}

TEST_CASE("VariableSizeFrame setData resizes and updates DLC", "[var][data]") {
	auto frame = FrameFactory::createVariableFrame(FrameType::EXT_VAR, FrameFmt::DATA_VAR, 0);
	frame->setID(0x1ABCDE0);
	frame->setData({0xAA,0xBB,0xCC});
	REQUIRE(frame->getDLC()==3);
	frame->setData({});
	REQUIRE(frame->getDLC()==0);
}
