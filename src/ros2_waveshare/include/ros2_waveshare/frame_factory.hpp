/**
 * @file frame_factory.hpp
 * @brief Factory class for creating USB-CAN frame objects
 * This file defines the FrameFactory class that provides static methods to create
 * frame objects using the builder pattern. It ensures proper initialization and
 * validation of frame parameters.
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

#include "adapter_base_frame.hpp"
#include "fixed_size_frame.hpp"
#include "variable_size_frame.hpp"
#include "usb_can_common.hpp"

#include <memory>
#include <vector>

namespace USBCANBridge
{

/**
 * @brief Factory class for creating frame objects
 * This class provides static methods to create frame objects using the builder pattern.
 * It ensures proper initialization and validation of frame parameters.
 */
class FrameFactory {
public:
    /**
     * @brief Create a fixed-size (20 byte) frame instance.
     * @param type Frame meta type (DATA_FIXED or CONF_FIXED)
     * @param frame_type Identifier type (STD_FIXED or EXT_FIXED)
     * @param fmt Frame format (DATA_FIXED or REMOTE_FIXED)
     * @return Newly allocated FixedSizeFrame unique_ptr
     * @throws std::invalid_argument if parameters are invalid.
     */
    static std::unique_ptr<FixedSizeFrame> createFixedFrame(
        Type type, FrameType frame_type, FrameFmt fmt);

    /**
     * @brief Create a variable-length frame instance.
     * @param frame_type STD_VAR or EXT_VAR
     * @param frame_fmt DATA_VAR or REMOTE_VAR
     * @param dlc Payload length (0..8)
     * @return Newly allocated VariableSizeFrame unique_ptr
     * @throws std::invalid_argument if parameters are invalid.
     */
    static std::unique_ptr<VariableSizeFrame> createVariableFrame(
        FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc = 0);

    /**
     * @brief Create a frame by parsing raw adapter bytes.
     *
     * Auto-detects fixed or variable frame based on size and sentinel bytes.
     * @param raw_data Raw byte vector beginning with START_BYTE.
     * @return Concrete frame instance (FixedSizeFrame or VariableSizeFrame)
     * @throws std::invalid_argument if parsing fails or data invalid.
     */
    static std::unique_ptr<AdapterBaseFrame> createFrameFromData(
        const std::vector<uint8_t>& raw_data);
};

} // namespace USBCANBridge
