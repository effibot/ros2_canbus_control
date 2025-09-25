#include "fixed_frame.hpp"
#include "config_frame.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <thread>
#include <chrono>

using namespace USBCANBridge;

static int adapter_init(const char* tty_device, int baudrate) {
    int tty_fd, result;
    termios2 tio;

    tty_fd = open(tty_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (tty_fd == -1) {
        fprintf(stderr, "open(%s) failed: %s\n", tty_device,
            strerror(errno));
        return -1;
    }

    result = ioctl(tty_fd, TCGETS2, &tio);
    if (result == -1) {
        fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
        close(tty_fd);
        return -1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag = BOTHER | CS8 | CSTOPB;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    result = ioctl(tty_fd, TCSETS2, &tio);
    if (result == -1) {
        fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
        close(tty_fd);
        return -1;
    }

    return tty_fd;
}

int main() {
    FixedFrame fixed_frame;
    ConfigFrame config_frame;

    // print the storage
    std::cout << "FixedFrame storage: ";
    std::cout << bytes_to_hex_string(fixed_frame.impl_serialize().value) << std::endl;
    std::cout << "ConfigFrame storage: ";
    std::cout << bytes_to_hex_string(config_frame.impl_serialize().value) << std::endl;

    // Example usage of FixedFrame
    auto clear_res = fixed_frame.impl_clear();
    if (clear_res.fail()) {
        std::cerr << "Failed to clear FixedFrame: " << clear_res.to_string() << std::endl;
        return -1;
    }

    auto type_res = fixed_frame.impl_get_type();
    if (type_res.fail()) {
        std::cerr << "Failed to get type from FixedFrame: " << type_res.to_string() << std::endl;
        return -1;
    }

    auto serialize_res = fixed_frame.impl_serialize();
    if (serialize_res.fail()) {
        std::cerr << "Failed to serialize FixedFrame: " << serialize_res.to_string() << std::endl;
        return -1;
    }

    // Example usage of ConfigFrame
    auto config_clear_res = config_frame.impl_clear();
    if (config_clear_res.fail()) {
        std::cerr << "Failed to clear ConfigFrame: " << config_clear_res.to_string() << std::endl;
        return -1;
    }

    auto config_type_res = config_frame.impl_get_type();
    if (config_type_res.fail()) {
        std::cerr << "Failed to get type from ConfigFrame: " << config_type_res.to_string() <<
            std::endl;
        return -1;
    }



    // try to set some values for fixed frame to have storage like:
    // AA 55 01 01 01 23 01 00 00 08 11 22 33 44 55 66 77 00 93
    fixed_frame.set_type(Type::DATA_FIXED);
    fixed_frame.set_frame_type(FrameType::STD_FIXED);
    fixed_frame.set_frame_fmt(FrameFmt::DATA_FIXED);
    frame_traits_t<FixedFrame>::IDPair id = {{std::byte{0x23}, std::byte{0x01}}, 2};
    fixed_frame.impl_set_id(id);
    fixed_frame.impl_set_dlc(std::byte{0x08});
    frame_traits_t<FixedFrame>::PayloadPair data = {{std::byte{0x11}, std::byte{0x22},
        std::byte{0x33}, std::byte{0x44}, std::byte{0x55}, std::byte{0x66}, std::byte{0x77},
        std::byte{0x88}}, 8};
    fixed_frame.impl_set_data(data);
    auto fixed_serialize_res = fixed_frame.impl_serialize();
    if (fixed_serialize_res.fail()) {
        std::cerr << "Failed to serialize FixedFrame after setting values: " <<
            fixed_serialize_res.to_string() << std::endl;
        return -1;
    }
    std::cout << "FixedFrame after setting values: ";
    std::cout << bytes_to_hex_string(fixed_serialize_res.value) << std::endl;

    // wait for user input, if enter is pressed, send the frame to the opened device
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();
    // send the frame to the opened ttyUSB device
    // Open USB device (placeholder for device name)
    const std::string device_name = "/dev/ttyUSB0"; // Edit this to your device
    int fd = adapter_init(device_name.c_str(), 2000000);
    if (fd == -1) {
        std::cerr << "Failed to initialize adapter" << std::endl;
        return -1;
    }

    // Configure device settings
    config_frame.set_baud_rate(CANBaud::SPEED_1000K);
    config_frame.set_can_mode(CANMode::NORMAL);
    config_frame.set_type(Type::CONF_FIXED);
    config_frame.set_frame_type(FrameType::STD_FIXED);
    auto config_serialize_res = config_frame.impl_serialize();
    if (config_serialize_res.fail()) {
        std::cerr << "Failed to serialize ConfigFrame after setting values: " <<
            config_serialize_res.to_string() << std::endl;
        close(fd);
        return -1;
    }
    // Send the serialized config frame
    auto config_data = config_serialize_res.value;
    ssize_t config_bytes_written = write(fd, config_data.data(), config_data.size());
    if (config_bytes_written == -1) {
        std::cerr << "Failed to write config to device" << std::endl;
        close(fd);
        return -1;
    }
    std::cout << "Successfully wrote " << config_bytes_written << " bytes of config to "
              << device_name << std::endl;

    // Send the serialized frame
    while (true) {

        auto frame_data = fixed_serialize_res.value;
        ssize_t bytes_written = write(fd, frame_data.data(), frame_data.size());
        if (bytes_written == -1) {
            std::cerr << "Failed to write to device" << std::endl;
            close(fd);
            return -1;
        }
        std::cout << "Successfully wrote " << bytes_written << " bytes to " << device_name <<
            std::endl;
        // sleep for 3 seconds
        std::this_thread::sleep_for(std::chrono::seconds(3));

    }


    close(fd);
    return 0;
}