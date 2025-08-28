// usb_can_driver.cpp
// Implementazione del driver USB-CAN per Waveshare CAN-USB-A
// Usa ROS2 (rclcpp) per logging e pubblicazione dei frame ricevuti.
//
// Compilare con: g++ -std=c++17 -O2 -pthread `pkg-config --cflags --libs rclcpp` usb_can_driver.cpp -o usb_can_driver
//
// Nota: richiede linux headers per termios2 (asm/termbits.h). Se non disponibili, sostituire con impostazione seriale alternativa.

#include "usb_can_driver.hpp" // il tuo header: assicurati che definisca CANFrame e namespace usb_can_bridge
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/termbits.h> // TCGETS2 / TCSETS2, struct termios2
#include <errno.h>
#include <string.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

namespace usb_can_bridge {

// --- Helpers / definizioni locali ---
static uint8_t compute_checksum(const uint8_t *data, size_t len) {
	unsigned sum = 0;
	for (size_t i = 0; i < len; ++i) sum += data[i];
	return static_cast<uint8_t>(sum & 0xFF);
}

// Il formato wire usato dal Waveshare (come nel canusb.c):
// Command frames start with 0xaa 0x55 and have fixed length 20 (with checksum).
// Data frames start with 0xaa and byte1 high nibble 0xC (i.e. (byte1 >> 4) == 0xC).
// Byte layout for data frame sent by adapter:
// [0]=0xaa [1]=info (bits incl. ext/std + dlc) [2]=id_lsb [3]=id_msb [4..] data... [last]=0x55

// CANFrame type expected (assicurati che il tuo header corrisponda):
// struct CANFrame {
//   uint32_t can_id; uint8_t dlc; uint8_t flags; uint8_t data[8]; uint8_t iface; uint64_t timestamp;
//   static constexpr uint8_t FLAG_EFF = 0x01; static constexpr uint8_t FLAG_RTR = 0x02; static constexpr uint8_t FLAG_ERR = 0x04;
//   ...
// };

// small helper to build SDO upload request (read 0x6041 sub 0x00)
static std::array<uint8_t,8> make_sdo_upload_request() {
	std::array<uint8_t,8> d{};
	d[0] = 0x40; // CCS=2 upload initiate
	d[1] = 0x41; // index low (0x6041)
	d[2] = 0x60; // index high
	d[3] = 0x00; // subindex
	// d[4..7] = 0
	return d;
}

static CANFrame parse_wire_frame(const std::vector<uint8_t>& wire, uint8_t iface_index, uint64_t ts_ns) {
	// wire: full frame as read by frame_recv: [0xaa, info, id_lsb, id_msb, data..., 0x55]
	CANFrame f;
	f.iface = iface_index;
	f.timestamp = ts_ns;
	if (wire.size() < 6) { // minimal
		f.flags = CANFrame::FLAG_ERR;
		return f;
	}
	uint8_t info = wire[1];
	uint8_t dlc = info & 0x0F;
	bool extended = (info & 0x20) != 0;
	f.dlc = dlc;
	f.flags = 0;
	if (extended) f.flags |= CANFrame::FLAG_EFF;
	// reconstruct id: id_lsb, id_msb (11-bit in standard)
	uint16_t id_lsb = wire[2];
	uint16_t id_msb = wire[3];
	uint32_t id = (id_msb << 8) | id_lsb; // as in canusb.c they print frame[3], frame[2]
	f.can_id = id;
	// data bytes are from wire[4] up to wire[4+dlc-1]
	for (uint8_t i=0; i<dlc && (4+i) < wire.size()-1; ++i) {
		f.data[i] = wire[4+i];
	}
	return f;
}

static std::vector<uint8_t> build_wire_frame_send(CANFrame const& f, bool use_extended=false) {
	// produce bytes for write to serial using Waveshare format
	std::vector<uint8_t> out;
	out.reserve(13);
	out.push_back(0xAA);
	uint8_t info = 0xC0; // bit7/6 = 1
	if (use_extended) info |= 0x20;
	info &= 0xEF; // data frame
	info |= (f.dlc & 0x0F);
	out.push_back(info);
	// ID: need to split little-endian (lsb first) as canusb expects
	uint16_t id16 = static_cast<uint16_t>(f.can_id & 0xFFFF);
	uint8_t id_lsb = static_cast<uint8_t>(id16 & 0xFF);
	uint8_t id_msb = static_cast<uint8_t>((id16 >> 8) & 0xFF);
	out.push_back(id_lsb);
	out.push_back(id_msb);
	for (int i=0; i<f.dlc; i++) out.push_back(f.data[i]);
	out.push_back(0x55);
	return out;
}

// --- USBCANDriver implementation ---

USBCANDriver::USBCANDriver(rclcpp::Node::SharedPtr node, std::string device_path, int serial_baud, int can_bitrate)
	: node_(node),
	logger_(node->get_logger()),
	device_path_(std::move(device_path)),
	serial_baud_(serial_baud),
	can_speed_bps_(can_bitrate),
	running_(false),
	tty_fd_(-1)
{
	// publisher: raw frames as byte arrays (id msb/lsb + dlc + data...)
	publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("usb_can/frames", 10);
}

USBCANDriver::~USBCANDriver() {
	stop();
}

bool USBCANDriver::start() {
	std::lock_guard<std::mutex> lk(start_stop_mtx_);
	if (running_) return true;
	// open serial / configure
	tty_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (tty_fd_ == -1) {
		RCLCPP_ERROR(logger_, "Failed to open serial device %s: %s", device_path_.c_str(), strerror(errno));
		return false;
	}
	// set baudrate using termios2 (like adapter_init in canusb.c)
	struct termios2 tio;
	if (ioctl(tty_fd_, TCGETS2, &tio) == -1) {
		RCLCPP_ERROR(logger_, "ioctl TCGETS2 failed: %s", strerror(errno));
		close(tty_fd_); tty_fd_ = -1;
		return false;
	}
	tio.c_cflag &= ~CBAUD;
	tio.c_cflag = BOTHER | CS8 | CSTOPB;
	tio.c_iflag = IGNPAR;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_ispeed = serial_baud_;
	tio.c_ospeed = serial_baud_;
	if (ioctl(tty_fd_, TCSETS2, &tio) == -1) {
		RCLCPP_ERROR(logger_, "ioctl TCSETS2 failed: %s", strerror(errno));
		close(tty_fd_); tty_fd_ = -1;
		return false;
	}

	// send adapter configuration command (similar to command_settings)
	if (!send_command_settings(can_speed_bps_)) {
		RCLCPP_ERROR(logger_, "Failed to send adapter command settings");
		close(tty_fd_); tty_fd_ = -1;
		return false;
	}

	running_ = true;
	reader_thread_ = std::thread(&USBCANDriver::reader_loop, this);
	worker_thread_ = std::thread(&USBCANDriver::worker_loop, this);
	RCLCPP_INFO(logger_, "USBCANDriver started on %s", device_path_.c_str());
	return true;
}

void USBCANDriver::stop() {
	std::lock_guard<std::mutex> lk(start_stop_mtx_);
	if (!running_) return;
	running_ = false;
	if (reader_thread_.joinable()) reader_thread_.join();
	if (worker_thread_.joinable()) worker_thread_.join();
	if (tty_fd_ != -1) {
		close(tty_fd_);
		tty_fd_ = -1;
	}
	RCLCPP_INFO(logger_, "USBCANDriver stopped");
}

// send adapter "command_settings" as in canusb.c
bool USBCANDriver::send_command_settings(int can_bitrate) {
	// construct 20-byte command frame as in canusb.c
	uint8_t frame[20];
	int idx = 0;
	frame[idx++] = 0xAA;
	frame[idx++] = 0x55;
	frame[idx++] = 0x12;
	// map bitrate int to Waveshare code. We mimic canusb_int_to_speed
	uint8_t speed_code = 0;
	switch (can_bitrate) {
	case 1000000: speed_code = 0x01; break;
	case 800000: speed_code = 0x02; break;
	case 500000: speed_code = 0x03; break;
	case 400000: speed_code = 0x04; break;
	case 250000: speed_code = 0x05; break;
	case 200000: speed_code = 0x06; break;
	case 125000: speed_code = 0x07; break;
	case 100000: speed_code = 0x08; break;
	case 50000: speed_code = 0x09; break;
	case 20000: speed_code = 0x0A; break;
	case 10000: speed_code = 0x0B; break;
	case 5000: speed_code = 0x0C; break;
	default: speed_code = 0x03; break;
	}
	frame[idx++] = speed_code;
	frame[idx++] = 0x01; // frame type: standard
	// next 8 bytes: filter IDs & mask (we put 0 -> no hardware filter)
	for (int i=0; i<8; i++) frame[idx++] = 0;
	// mode (silent/normal etc). Set normal 0x00
	frame[13] = 0x00;
	// then some reserved zero bytes (already zeroed)
	for (; idx < 19; ++idx) frame[idx] = 0;
	frame[19] = compute_checksum(&frame[2], 17);
	// write
	ssize_t w = write(tty_fd_, frame, sizeof(frame));
	if (w != (ssize_t)sizeof(frame)) {
		RCLCPP_ERROR(logger_, "Failed to write command settings: wrote %ld bytes (err=%s)", (long)w, strerror(errno));
		return false;
	}
	// adapter may reply; we ignore for now
	rclcpp::sleep_for(10ms);
	return true;
}

// low-level write of a wire frame to serial
bool USBCANDriver::write_wire(const std::vector<uint8_t>& buffer) {
	ssize_t n = write(tty_fd_, buffer.data(), buffer.size());
	if (n != (ssize_t)buffer.size()) {
		RCLCPP_ERROR(logger_, "Serial write failed: wrote %ld of %zu (%s)", (long)n, buffer.size(), strerror(errno));
		return false;
	}
	return true;
}

void USBCANDriver::enqueue_received_frame(const CANFrame& f) {
	{
		std::lock_guard<std::mutex> lk(rx_mtx_);
		rx_queue_.push(f);
	}
	rx_cv_.notify_one();
}

// reader loop: riceve bytes dalla seriale, sincronizza sui frame e pubblica su ROS topic
void USBCANDriver::reader_loop() {
	std::vector<uint8_t> buffer;
	buffer.reserve(64);
	uint8_t byte;
	while (running_) {
		ssize_t r = read(tty_fd_, &byte, 1);
		if (r == -1) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				std::this_thread::sleep_for(1ms);
				continue;
			} else {
				RCLCPP_ERROR(logger_, "Serial read error: %s", strerror(errno));
				break;
			}
		} else if (r == 0) {
			std::this_thread::sleep_for(1ms);
			continue;
		}
		buffer.push_back(byte);
		// try to detect complete frame (reuse logic from canusb.c simplified)
		if (buffer.size() >= 2) {
			// if not sync, drop until 0xaa
			if (buffer[0] != 0xAA) {
				// discard until we find 0xAA
				auto it = std::find(buffer.begin(), buffer.end(), 0xAA);
				if (it == buffer.end()) {
					buffer.clear();
					continue;
				} else {
					// erase leading bytes
					buffer.erase(buffer.begin(), it);
					if (buffer.size() < 2) continue;
				}
			}
			// if command frame (0xaa 0x55) length 20
			if (buffer[1] == 0x55) {
				if (buffer.size() >= 20) {
					// verify checksum
					uint8_t chk = compute_checksum(buffer.data()+2, 17);
					if (chk != buffer[19]) {
						RCLCPP_WARN(logger_, "Command frame checksum mismatch");
						buffer.erase(buffer.begin(), buffer.begin()+1); // try to resync slowly
						continue;
					}
					// ignore command frames for now
					buffer.erase(buffer.begin(), buffer.begin()+20);
					continue;
				} else {
					continue; // wait more
				}
			} else if ((buffer[1] >> 4) == 0xC) {
				// data frame: length = (buffer[1] & 0xF) + 5
				int expected = (buffer[1] & 0x0F) + 5;
				if ((int)buffer.size() >= expected) {
					// capture the frame bytes
					std::vector<uint8_t> frame_bytes(buffer.begin(), buffer.begin()+expected);
					// compute timestamp
					struct timespec ts;
					clock_gettime(CLOCK_MONOTONIC, &ts);
					uint64_t ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
					CANFrame f = parse_wire_frame(frame_bytes, /*iface*/ 0, ts_ns);
					// publish to ROS topic
					publish_frame_to_ros(f);
					// enqueue for internal consumers
					enqueue_received_frame(f);
					// erase processed bytes
					buffer.erase(buffer.begin(), buffer.begin()+expected);
					continue;
				} else {
					continue;
				}
			} else {
				// unknown pattern -- drop first byte to resync
				buffer.erase(buffer.begin());
			}
		}
	}
}

// publish raw frame as UInt8MultiArray: format [id_msb, id_lsb, dlc, data0..dataN-1]
void USBCANDriver::publish_frame_to_ros(const CANFrame& f) {
	std_msgs::msg::UInt8MultiArray msg;
	msg.data.reserve(3 + f.dlc);
	// store id as two bytes msb/lsb to match canusb.c print (frame[3], frame[2])
	uint16_t id16 = static_cast<uint16_t>(f.can_id & 0xFFFF);
	uint8_t id_lsb = static_cast<uint8_t>(id16 & 0xFF);
	uint8_t id_msb = static_cast<uint8_t>((id16 >> 8) & 0xFF);
	msg.data.push_back(id_msb);
	msg.data.push_back(id_lsb);
	msg.data.push_back(f.dlc);
	for (int i=0; i<f.dlc; i++) msg.data.push_back(f.data[i]);
	publisher_->publish(msg);
}

// worker loop: processes received frames (currently used by checkDriverStatus to wait/dispatch)
void USBCANDriver::worker_loop() {
	while (running_) {
		std::unique_lock<std::mutex> lk(rx_mtx_);
		rx_cv_.wait_for(lk, 100ms, [&]{
				return !rx_queue_.empty() || !running_;
			});
		while (!rx_queue_.empty()) {
			CANFrame f = rx_queue_.front();
			rx_queue_.pop();
			lk.unlock();
			// notify any waiting waiters: match by COB-ID
			{
				std::lock_guard<std::mutex> g(waiters_mtx_);
				// iterate copy of map keys
				for (auto &entry : waiters_) {
					uint32_t expected_cobid = entry.first;
					auto &vec = entry.second;
					// check if frame matches expected cobid (exact)
					if (f.can_id == expected_cobid) {
						// push to all waiters for that COB-ID
						for (auto &wptr : vec) {
							if (auto wp = wptr.lock()) {
								std::lock_guard<std::mutex> wlk(wp->mtx);
								wp->frames.push_back(f);
								wp->cv.notify_one();
							}
						}
					}
				}
			}
			lk.lock();
		}
	}
}

// send CAN frame via Waveshare wire format
bool USBCANDriver::send_can_frame(const CANFrame& f) {
	auto wire = build_wire_frame_send(f, (f.flags & CANFrame::FLAG_EFF) != 0);
	return write_wire(wire);
}

// Public API: checkDriverStatus(node_id)
// Returns true on success and writes statusword into out_status (16-bit)
bool USBCANDriver::checkDriverStatus(uint8_t node_id, uint16_t &out_status, std::chrono::milliseconds timeout) {
	if (!running_) {
		RCLCPP_ERROR(logger_, "Driver not running");
		return false;
	}
	// 1) build SDO upload request frame: COB-ID = 0x600 + node_id
	uint32_t cob_req = 0x600 + node_id;
	CANFrame fr;
	fr.can_id = cob_req;
	fr.dlc = 8;
	fr.flags = 0; // standard
	auto payload = make_sdo_upload_request();
	for (int i=0; i<8; i++) fr.data[i] = payload[i];

	// 2) create a waiter object and register on expected response COB-ID 0x580 + node_id
	uint32_t cob_resp = 0x580 + node_id;
	struct Waiter {
		std::mutex mtx;
		std::condition_variable cv;
		std::vector<CANFrame> frames;
	};
	auto waiter = std::make_shared<Waiter>();

	{
		std::lock_guard<std::mutex> g(waiters_mtx_);
		waiters_[cob_resp].push_back(std::weak_ptr<Waiter>(waiter));
	}

	// 3) send the request
	if (!send_can_frame(fr)) {
		RCLCPP_ERROR(logger_, "Failed to send SDO upload request to node %u", node_id);
		// unregister waiter
		std::lock_guard<std::mutex> g(waiters_mtx_);
		auto &vec = waiters_[cob_resp];
		vec.erase(std::remove_if(vec.begin(), vec.end(),
		                         [&](const std::weak_ptr<Waiter>& w){
				return w.expired() || w.lock() == waiter;
			}),
		          vec.end());
		return false;
	}

	// 4) wait for response or timeout
	{
		std::unique_lock<std::mutex> lk(waiter->mtx);
		bool got = waiter->cv.wait_for(lk, timeout, [&]{
				return !waiter->frames.empty();
			});
		if (!got) {
			RCLCPP_WARN(logger_, "Timeout waiting SDO response from node %u", node_id);
			// cleanup waiter
			std::lock_guard<std::mutex> g(waiters_mtx_);
			auto &vec = waiters_[cob_resp];
			vec.erase(std::remove_if(vec.begin(), vec.end(),
			                         [&](const std::weak_ptr<Waiter>& w){
					return w.expired() || w.lock() == waiter;
				}),
			          vec.end());
			return false;
		}
		// take first matching frame
		CANFrame resp = waiter->frames.front();
		// parse SDO upload response: data[0] command, [1..3] index etc, [4..] data
		if (resp.dlc < 6) {
			RCLCPP_ERROR(logger_, "SDO response too short from node %u", node_id);
			return false;
		}
		uint8_t cmd = resp.data[0];
		// expedited with size indicated and 2 bytes => typical cmd nibble 0x4B/0x43 etc. We'll accept any with 2 bytes
		// find statusword in bytes 4 (LSB) and 5 (MSB)
		out_status = static_cast<uint16_t>((uint16_t)resp.data[4] | ((uint16_t)resp.data[5] << 8));
		// unregister waiter
		std::lock_guard<std::mutex> g(waiters_mtx_);
		auto &vec = waiters_[cob_resp];
		vec.erase(std::remove_if(vec.begin(), vec.end(),
		                         [&](const std::weak_ptr<Waiter>& w){
				return w.expired() || w.lock() == waiter;
			}),
		          vec.end());
		return true;
	}
}

} // namespace usb_can_bridge
