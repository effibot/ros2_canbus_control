/**
 * @file test_frames.cpp
 * @brief Unit tests for USB-CAN frame system
 * @author Andrea Efficace
 * @date September 2025
 */

#include <gtest/gtest.h>
#include "ros2_waveshare/frames/usb_can_frames.hpp"

using namespace usb_can_bridge;

class USBCANFrameTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code for each test
    }
    
    void TearDown() override {
        // Cleanup code for each test
    }
    
    // Helper function to compare vectors
    bool vectorsEqual(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
        return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
    }
};

// Test Settings Frame
TEST_F(USBCANFrameTest, SettingsFrameCreationAndSerialization) {
    SettingsFrame settings(USBCANBaud::SPEED_500K, USBCANFrameFmt::VARIABLE, USBCANMode::NORMAL);
    
    EXPECT_EQ(settings.getType(), USBCANFrameType::SETTING_FRAME);
    EXPECT_EQ(settings.getFrameSize(), 20);
    EXPECT_TRUE(settings.isValid());
    
    auto data = settings.serialize();
    EXPECT_EQ(data.size(), 20);
    EXPECT_EQ(data[0], 0xAA); // Start byte
    EXPECT_EQ(data[1], 0x55); // Settings frame info
}

TEST_F(USBCANFrameTest, SettingsFrameConfiguration) {
    SettingsFrame settings;
    
    settings.setBaudRate(USBCANBaud::SPEED_1M);
    settings.setCANMode(USBCANMode::LOOPBACK);
    settings.setFilterID(0x12345678);
    settings.setMaskID(0xFFFFFFFF);
    
    EXPECT_EQ(settings.getBaudRate(), USBCANBaud::SPEED_1M);
    EXPECT_EQ(settings.getCANMode(), USBCANMode::LOOPBACK);
    EXPECT_EQ(settings.getFilterID(), 0x12345678);
    EXPECT_EQ(settings.getMaskID(), 0xFFFFFFFF);
}

// Test Standard Variable Frame
TEST_F(USBCANFrameTest, StandardVariableFrameCreationAndSerialization) {
    StandardVariableFrame frame;
    frame.setCANID(0x123);
    frame.setCANData({0x01, 0x02, 0x03, 0x04});
    
    EXPECT_EQ(frame.getType(), USBCANFrameType::STANDARD_VARIABLE);
    EXPECT_EQ(frame.getCANID(), 0x123);
    EXPECT_TRUE(vectorsEqual(frame.getCANData(), {0x01, 0x02, 0x03, 0x04}));
    EXPECT_TRUE(frame.isValid());
    
    auto data = frame.serialize();
    EXPECT_EQ(data[0], 0xAA); // Start byte
    EXPECT_EQ(data.back(), 0x55); // End byte
    EXPECT_EQ(data.size(), 2 + 2 + 4 + 1); // start + info + id(2) + data(4) + end
}

// Test Extended Variable Frame
TEST_F(USBCANFrameTest, ExtendedVariableFrameCreationAndSerialization) {
    ExtendedVariableFrame frame;
    frame.setCANID(0x12345678);
    frame.setCANData({0xAA, 0xBB, 0xCC});
    frame.setRTR(true);
    
    EXPECT_EQ(frame.getType(), USBCANFrameType::EXTENDED_VARIABLE);
    EXPECT_EQ(frame.getCANID(), 0x12345678);
    EXPECT_TRUE(vectorsEqual(frame.getCANData(), {0xAA, 0xBB, 0xCC}));
    EXPECT_TRUE(frame.isRTR());
    EXPECT_TRUE(frame.isValid());
    
    auto data = frame.serialize();
    EXPECT_EQ(data[0], 0xAA); // Start byte
    EXPECT_EQ(data.back(), 0x55); // End byte
    EXPECT_EQ(data.size(), 2 + 4 + 3 + 1); // start + info + id(4) + data(3) + end
}

// Test Standard Fixed 20-byte Frame
TEST_F(USBCANFrameTest, StandardFixed20FrameCreationAndSerialization) {
    StandardFixed20Frame frame;
    frame.setCANID(0x456);
    frame.setCANData({0x11, 0x22, 0x33});
    frame.setTimestamp(0x12345678);
    
    EXPECT_EQ(frame.getType(), USBCANFrameType::STANDARD_FIXED_20);
    EXPECT_EQ(frame.getFrameSize(), 20);
    EXPECT_EQ(frame.getCANID(), 0x456);
    EXPECT_EQ(frame.getTimestamp(), 0x12345678);
    EXPECT_TRUE(frame.isValid());
    
    auto data = frame.serialize();
    EXPECT_EQ(data.size(), 20);
    EXPECT_EQ(data[0], 0xAA); // Start byte
    EXPECT_EQ(data[1], 0x01); // Standard fixed frame info
}

// Test Extended Fixed 20-byte Frame
TEST_F(USBCANFrameTest, ExtendedFixed20FrameCreationAndSerialization) {
    ExtendedFixed20Frame frame;
    frame.setCANID(0x1ABCDEF0);
    frame.setCANData({0xDE, 0xAD, 0xBE, 0xEF});
    frame.setTimestamp(0x87654321);
    
    EXPECT_EQ(frame.getType(), USBCANFrameType::EXTENDED_FIXED_20);
    EXPECT_EQ(frame.getFrameSize(), 20);
    EXPECT_EQ(frame.getCANID(), 0x1ABCDEF0);
    EXPECT_EQ(frame.getTimestamp(), 0x87654321);
    EXPECT_TRUE(frame.isValid());
    
    auto data = frame.serialize();
    EXPECT_EQ(data.size(), 20);
    EXPECT_EQ(data[0], 0xAA); // Start byte
    EXPECT_EQ(data[1], 0x02); // Extended fixed frame info
}

// Test Factory Pattern
TEST_F(USBCANFrameTest, FactoryPatternCreation) {
    FrameFactory factory;
    
    auto settings = factory.createFrame(USBCANFrameType::SETTING_FRAME);
    auto std_var = factory.createFrame(USBCANFrameType::STANDARD_VARIABLE);
    auto ext_fixed = factory.createFrame(USBCANFrameType::EXTENDED_FIXED_20);
    
    ASSERT_NE(settings, nullptr);
    ASSERT_NE(std_var, nullptr);
    ASSERT_NE(ext_fixed, nullptr);
    
    EXPECT_EQ(settings->getType(), USBCANFrameType::SETTING_FRAME);
    EXPECT_EQ(std_var->getType(), USBCANFrameType::STANDARD_VARIABLE);
    EXPECT_EQ(ext_fixed->getType(), USBCANFrameType::EXTENDED_FIXED_20);
}

// Test Builder Pattern
TEST_F(USBCANFrameTest, BuilderPatternCreation) {
    // Test standard frame building
    auto std_frame = FrameBuilder()
        .setFrameType(USBCANFrameType::STANDARD_VARIABLE)
        .setCANID(0x789)
        .setCANData({0x55, 0x66, 0x77})
        .setRTR(false)
        .build();
    
    ASSERT_NE(std_frame, nullptr);
    EXPECT_EQ(std_frame->getType(), USBCANFrameType::STANDARD_VARIABLE);
    
    // Test settings frame building
    auto settings = FrameBuilder()
        .setFrameType(USBCANFrameType::SETTING_FRAME)
        .setBaudRate(USBCANBaud::SPEED_250K)
        .setCANMode(USBCANMode::NORMAL)
        .build();
    
    ASSERT_NE(settings, nullptr);
    EXPECT_EQ(settings->getType(), USBCANFrameType::SETTING_FRAME);
}

// Test Convenience Builders
TEST_F(USBCANFrameTest, ConvenienceBuilders) {
    auto std_frame = FrameBuilders::createStandardCANFrame(0x123, {0x01, 0x02});
    auto ext_frame = FrameBuilders::createExtendedCANFrame(0x12345678, {0xAA, 0xBB});
    auto rtr_frame = FrameBuilders::createRTRFrame(0x456, false);
    auto settings = FrameBuilders::createSettingsFrame();
    
    ASSERT_NE(std_frame, nullptr);
    ASSERT_NE(ext_frame, nullptr);
    ASSERT_NE(rtr_frame, nullptr);
    ASSERT_NE(settings, nullptr);
    
    EXPECT_EQ(std_frame->getType(), USBCANFrameType::STANDARD_VARIABLE);
    EXPECT_EQ(ext_frame->getType(), USBCANFrameType::EXTENDED_VARIABLE);
    EXPECT_EQ(rtr_frame->getType(), USBCANFrameType::STANDARD_VARIABLE);
    EXPECT_EQ(settings->getType(), USBCANFrameType::SETTING_FRAME);
}

// Test Serialization/Deserialization Round Trip
TEST_F(USBCANFrameTest, SerializationDeserializationRoundTrip) {
    FrameFactory factory;
    
    // Create original frame
    auto original = FrameBuilder()
        .setFrameType(USBCANFrameType::STANDARD_VARIABLE)
        .setCANID(0xABC)
        .setCANData({0x10, 0x20, 0x30, 0x40})
        .build();
    
    // Serialize
    auto data = original->serialize();
    
    // Deserialize
    auto deserialized = factory.createFrameFromData(data);
    
    ASSERT_NE(deserialized, nullptr);
    EXPECT_EQ(deserialized->getType(), original->getType());
    EXPECT_TRUE(deserialized->isValid());
    
    // Compare serialized data
    auto deserialized_data = deserialized->serialize();
    EXPECT_TRUE(vectorsEqual(data, deserialized_data));
}

// Test Error Handling
TEST_F(USBCANFrameTest, ErrorHandling) {
    // Test builder error handling
    EXPECT_THROW({
        FrameBuilder builder;
        builder.setCANID(0x123); // Should throw - no frame type set
    }, std::runtime_error);
    
    // Test invalid data deserialization
    FrameFactory factory;
    std::vector<uint8_t> invalid_data = {0xFF, 0xFF, 0xFF};
    auto invalid_frame = factory.createFrameFromData(invalid_data);
    EXPECT_EQ(invalid_frame, nullptr);
    
    // Test too large CAN data
    StandardVariableFrame frame;
    std::vector<uint8_t> large_data(10, 0xFF); // Too large for CAN
    frame.setCANData(large_data);
    EXPECT_LE(frame.getCANData().size(), 8); // Should be truncated
}

// Test Checksum Validation for 20-byte frames
TEST_F(USBCANFram