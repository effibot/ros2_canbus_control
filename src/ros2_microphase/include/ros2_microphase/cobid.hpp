/**
 * @file cobid.hpp
 * @author effibot (andrea.efficace1@gmail.com)
 * @brief Describe the Communication Object IDs used by Microphase.
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

    // === Function Codes ===

    /**
     * @brief COB-IDs for the mini-TractionPower Driver.
     */
    enum class FunctionCode : uint16_t {
        NMT        = 0x000,
        SYNC       = 0x080,
        EMCY       = 0x080,// Base, actual = 0x080 + node_id
        TIMESTAMP  = 0x100,
        TPDO1      = 0x180,// Base, actual = 0x180 + node_id
        RPDO1      = 0x200,// Base, actual = 0x200 + node_id
        TPDO2      = 0x280,
        RPDO2      = 0x300,
        TPDO3      = 0x380,
        RPDO3      = 0x400,
        TPDO4      = 0x480,
        RPDO4      = 0x500,
        SDO_TX     = 0x580,// SDO response (slave → master)
        SDO_RX     = 0x600,// SDO request (master → slave)
        HEARTBEAT  = 0x700
    };

    static constexpr uint16_t fncode(FunctionCode fc, uint8_t node_id) {
        return static_cast<uint16_t>(fc) + static_cast<uint16_t>(node_id);
    }

    
}   // namespace microphase
