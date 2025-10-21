/**
 * @file sdo.hpp
 * @author effibot (andrea.efficace1@gmail.com)
 * @brief Service Data Objects (SDO) definitions for Microphase.
 * @version 0.1
 * @date 2025-10-21
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace microphase {
    // === SDO Command Specifiers ===

    /**
     * @brief SDO Request Command Specifiers
     *
     */
    enum class SDORequest : uint8_t {
        // Client → Server (Download = write to device)
        DOWNLOAD_INITIATE = 0x20,// Start write sequence
        DOWNLOAD_SEGMENT = 0x00,// Continue write (bits 0-2 vary)
        DOWNLOAD_INITIATE_EXPEDITED = 0x23,// ≤4 bytes, size indicated (n=1)

        // Server → Client (Upload = read from device)
        UPLOAD_INITIATE = 0x40,// Start read sequence
        UPLOAD_SEGMENT = 0x60,// Continue read
        UPLOAD_INITIATE_EXPEDITED = 0x43,// ≤4 bytes response (n=1)

    }

    /**
     * @brief SDO Response Command Specifiers
     *
     * - `Download` stands for write to device, tipically from Server to Client
     * - `Upload` stands for read from device, tipically from Client to Server
     */
    enum class SDOResponse : uint8_t {
        DOWNLOAD_INITIATE = 0x60,// Acknowledge write
        UPLOAD_INITIATE = 0x40,// Acknowledge read
        DOWNLOAD_SEGMENT = 0x20,// Acknowledge write segment
        UPLOAD_SEGMENT = 0x00,// Send read segment
        ABORT_ERROR = 0x80// Error
    };

    // === SDO Abort Codes ===
    /**
     * @brief SDO Abort Codes
     *
     */
    enum class SDOAbortCode : uint32_t {
        TOGGLE_BIT_NOT_CHANGED         = 0x05030000,
        SDO_PROTOCOL_TIMEOUT           = 0x05040000,
        INVALID_COMMAND_SPECIFIER      = 0x05040001,
        INVALID_BLOCK_SIZE             = 0x05040002,
        INVALID_SEQUENCE_NUMBER        = 0x05040003,
        CRC_ERROR                      = 0x05040004,
        OUT_OF_MEMORY                  = 0x05040005,
        UNSUPPORTED_ACCESS             = 0x06010000,
        WRITE_ONLY_OBJECT              = 0x06010001,
        READ_ONLY_OBJECT               = 0x06010002,
        OBJECT_NOT_EXIST               = 0x06020000,
        OBJECT_CANNOT_BE_MAPPED        = 0x06040041,
        PDO_LENGTH_EXCEEDED            = 0x06040042,
        PARAMETER_INCOMPATIBILITY      = 0x06040043,
        INTERNAL_INCOMPATIBILITY       = 0x06040047,
        HARDWARE_ERROR                 = 0x06060000,
        DATA_TYPE_MISMATCH             = 0x06070010,
        DATA_TYPE_TOO_HIGH             = 0x06070012,
        DATA_TYPE_TOO_LOW              = 0x06070013,
        SUBINDEX_NOT_EXIST             = 0x06090011,
        VALUE_RANGE_EXCEEDED           = 0x06090030,
        VALUE_TOO_HIGH                 = 0x06090031,
        VALUE_TOO_LOW                  = 0x06090032,
        GENERAL_ERROR                  = 0x08000000,
        DATA_CANNOT_BE_TRANSFERRED     = 0x08000020,
        DATA_CANNOT_BE_TRANSFERRED_LOCAL = 0x08000021,
        DATA_CANNOT_BE_TRANSFERRED_STATE = 0x08000022
    };

} // namespace microphase