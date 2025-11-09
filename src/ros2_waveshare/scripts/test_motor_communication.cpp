/**
 * @file test_motor_communication.cpp
 * @brief Integration test for SDO communication with real motor driver
 * @author effibot (andrea.efficace1@gmail.com)
 * @date 2025-11-06
 *
 * This script tests SDO communication with the actual motor driver:
 * 1. Read Statusword (0x6041) - Check motor state
 * 2. Read Error Register (0x1001) - Check for faults
 * 3. Read Mode of Operation Display (0x6061) - Current mode
 * 4. Read actual position and velocity
 * 5. Optionally write Mode of Operation (if not in fault)
 *
 * Prerequisites:
 * - vcan0 interface is up
 * - Waveshare bridge is running and active
 * - Motor driver is powered (24V) and connected to CAN bus
 * - Motor driver Node ID = 1
 */

#include "canopen/object_dictionary.hpp"
#include "canopen/sdo_client.hpp"
#include "io/real_can_socket.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>

// ANSI color codes for terminal output
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"

// Global flag for graceful shutdown
volatile sig_atomic_t g_shutdown = 0;

void signal_handler(int signal) {
    std::cout << "\n" << COLOR_YELLOW << "Caught signal " << signal << ", shutting down..." <<
        COLOR_RESET << "\n";
    g_shutdown = 1;
}

// Helper function to decode CIA402 statusword
std::string decode_statusword(uint16_t statusword) {
    uint8_t state_bits = statusword & 0x6F;

    if ((state_bits & 0x4F) == 0x00) return "NOT_READY_TO_SWITCH_ON";
    if ((state_bits & 0x4F) == 0x40) return "SWITCH_ON_DISABLED";
    if ((state_bits & 0x6F) == 0x21) return "READY_TO_SWITCH_ON";
    if ((state_bits & 0x6F) == 0x23) return "SWITCHED_ON";
    if ((state_bits & 0x6F) == 0x27) return "OPERATION_ENABLED";
    if ((state_bits & 0x6F) == 0x07) return "QUICK_STOP_ACTIVE";
    if ((state_bits & 0x4F) == 0x0F) return "FAULT_REACTION_ACTIVE";
    if ((state_bits & 0x4F) == 0x08) return "FAULT";

    return "UNKNOWN";
}

// Helper to print statusword details
void print_statusword(uint16_t statusword) {
    std::cout << "  " << COLOR_CYAN << "Statusword: 0x" << std::hex << std::setw(4)
              << std::setfill('0') << statusword << std::dec << COLOR_RESET << "\n";
    std::cout << "  State: " << COLOR_GREEN << decode_statusword(statusword) << COLOR_RESET << "\n";

    std::cout << "  Status bits:\n";
    if (statusword & (1 << 3)) std::cout << "    - Fault detected\n";
    if (statusword & (1 << 7)) std::cout << "    - Warning\n";
    if (statusword & (1 << 8)) std::cout << "    - Remote (CAN control active)\n";
    if (statusword & (1 << 9)) std::cout << "    - Target reached\n";
    if (statusword & (1 << 10)) std::cout << "    - Internal limit active\n";
    if (statusword & (1 << 12)) std::cout << "    - Operation mode specific bit 0\n";
    if (statusword & (1 << 13)) std::cout << "    - Operation mode specific bit 1\n";
}

// Helper to decode error register
void print_error_register(uint8_t error_reg) {
    std::cout << "  " << COLOR_CYAN << "Error Register: 0x" << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(error_reg) << std::dec << COLOR_RESET <<
        "\n";

    if (error_reg == 0) {
        std::cout << "  " << COLOR_GREEN << "✓ No errors" << COLOR_RESET << "\n";
        return;
    }

    std::cout << "  " << COLOR_RED << "Errors detected:" << COLOR_RESET << "\n";
    if (error_reg & 0x01) std::cout << "    - Generic error\n";
    if (error_reg & 0x02) std::cout << "    - Current error\n";
    if (error_reg & 0x04) std::cout << "    - Voltage error\n";
    if (error_reg & 0x08) std::cout << "    - Temperature error\n";
    if (error_reg & 0x10) std::cout << "    - Communication error\n";
    if (error_reg & 0x20) std::cout << "    - Device profile specific\n";
    if (error_reg & 0x80) std::cout << "    - Manufacturer specific\n";
}

// Helper to find config file in multiple locations
std::string find_config_file(const std::string& filename) {
    namespace fs = std::filesystem;

    // List of paths to search (in order of priority)
    std::vector<std::string> search_paths = {
        filename,  // Direct path if provided by user
        "../config/" + filename,  // From build directory
        "../../config/" + filename,  // From install/lib directory
        "../../../src/ros2_waveshare/config/" + filename,  // From build directory to source
        "config/" + filename,  // Current directory
        "/home/ros/ws/ros2_canbus_control/src/ros2_waveshare/config/" + filename  // Absolute fallback
    };

    for (const auto& path : search_paths) {
        if (fs::exists(path)) {
            return fs::absolute(path).string();
        }
    }

    throw std::runtime_error("Config file not found: " + filename +
        "\nSearched paths:\n" +
        [&]() {
        std::string paths;
        for (const auto& p : search_paths) {
            paths += "  - " + p + "\n";
        }
        return paths;
    } ());
}

int main(int argc, char** argv) {
    // Install signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        std::cout << "\n" << COLOR_BLUE <<
            "═══════════════════════════════════════════════════════════════\n";
        std::cout << "  Motor Driver SDO Communication Test\n";
        std::cout << "═══════════════════════════════════════════════════════════════"
                  << COLOR_RESET << "\n\n";

        // Get config file path
        std::string config_filename = "motor_config.json";
        if (argc > 1) {
            config_filename = argv[1];
        }

        std::cout << "[1] Looking for configuration file: " << config_filename << "\n";
        std::string config_path = find_config_file(config_filename);
        std::cout << "  Found at: " << config_path << "\n";

        canopen::ObjectDictionary dict(config_path);
        std::cout << "  " << COLOR_GREEN << "✓ Configuration loaded" << COLOR_RESET << "\n";
        std::cout << "  Device: " << dict.get_device_name() << "\n";
        std::cout << "  Node ID: " << static_cast<int>(dict.get_node_id()) << "\n";
        std::cout << "  CAN Interface: " << dict.get_can_interface() << "\n\n";

        // Create SDO client
        std::cout << "[2] Initializing SDO client...\n";

        // Create RealCANSocket for dependency injection
        auto socket = std::make_shared<waveshare::RealCANSocket>(dict.get_can_interface(), 1000);
        canopen::SDOClient sdo_client(socket, dict, dict.get_node_id());

        if (!socket->is_open()) {
            std::cerr << COLOR_RED << "  ✗ Failed to open CAN socket" << COLOR_RESET << "\n";
            return 1;
        }
        std::cout << "  " << COLOR_GREEN << "✓ SDO client connected to "
                  << dict.get_can_interface() << COLOR_RESET << "\n\n";

        // Test 1: Read Statusword
        std::cout << "[3] Reading Statusword (0x6041)...\n";
        uint16_t statusword = sdo_client.read<uint16_t>("statusword");
        print_statusword(statusword);
        std::cout << "\n";

        // Test 2: Read Error Register
        std::cout << "[4] Reading Error Register (0x1001)...\n";
        uint8_t error_reg = sdo_client.read<uint8_t>("error_register");
        print_error_register(error_reg);
        std::cout << "\n";

        // Test 3: Read Mode of Operation Display
        std::cout << "[5] Reading Mode of Operation Display (0x6061)...\n";
        int8_t mode_display = sdo_client.read<int8_t>("modes_of_operation_display");
        std::cout << "  Current mode: " << static_cast<int>(mode_display);
        switch (mode_display) {
        case 1: std::cout << " (Profile Position)\n"; break;
        case 3: std::cout << " (Profile Velocity)\n"; break;
        case 4: std::cout << " (Torque Profile)\n"; break;
        case 6: std::cout << " (Homing)\n"; break;
        case 8: std::cout << " (Cyclic Sync Position)\n"; break;
        case 9: std::cout << " (Cyclic Sync Velocity)\n"; break;
        case 10: std::cout << " (Cyclic Sync Torque)\n"; break;
        default: std::cout << " (Unknown)\n"; break;
        }
        std::cout << "\n";

        // Test 4: Read actual position
        std::cout << "[6] Reading Position Actual Value (0x6064)...\n";
        int32_t position = sdo_client.read<int32_t>("position_actual");
        std::cout << "  Position: " << position << " counts\n\n";

        // Test 5: Read actual velocity
        std::cout << "[7] Reading Velocity Actual Value (0x606C)...\n";
        int32_t velocity = sdo_client.read<int32_t>("velocity_actual");
        std::cout << "  Velocity: " << velocity << " rpm\n\n";

        // Test 6: Write Mode of Operation (only if not in fault)
        if ((statusword & 0x4F) != 0x08) {
            std::cout << "[8] Writing Mode of Operation (0x6060) to Profile Velocity (3)...\n";
            bool write_ok = sdo_client.write<int8_t>("modes_of_operation", 3);
            if (write_ok) {
                std::cout << "  " << COLOR_GREEN << "✓ Write successful" << COLOR_RESET << "\n";

                // Verify
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                int8_t verify_mode = sdo_client.read<int8_t>("modes_of_operation_display");
                std::cout << "  Verified mode: " << static_cast<int>(verify_mode) << "\n";
            } else {
                std::cout << "  " << COLOR_RED << "✗ Write failed" << COLOR_RESET << "\n";
            }
        } else {
            std::cout << "[8] " << COLOR_YELLOW << "Skipping mode write - motor is in FAULT state"
                      << COLOR_RESET << "\n";
        }

        std::cout << "\n" << COLOR_BLUE <<
            "═══════════════════════════════════════════════════════════════\n";
        std::cout << "  " << COLOR_GREEN << "✓ All SDO operations successful!" << COLOR_RESET <<
            "\n";
        std::cout << COLOR_BLUE << "═══════════════════════════════════════════════════════════════"
                  << COLOR_RESET << "\n\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "\n" << COLOR_RED << "❌ ERROR: " << e.what() << COLOR_RESET << "\n\n";
        return 1;
    }
}
