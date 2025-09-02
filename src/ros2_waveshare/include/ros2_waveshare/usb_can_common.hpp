#pragma once

//* -- USB adapter constants and enums --*
// Basic definitions for common and constant values
enum class USBCANConst
{
	START_BYTE = 0xAA,
	MSG_HEADER = 0x55,
	DEF_FILTER0 = 0x00,
	DEF_FILTER1 = 0x00,
	DEF_FILTER2 = 0x00,
	DEF_FILTER3 = 0x00,
	DEF_MASK0 = 0x00,
	DEF_MASK1 = 0x00,
	DEF_MASK2 = 0x00,
	DEF_MASK3 = 0x00,
	RESERVED0 = 0x00,
	RESERVED1 = 0x00,
	RESERVED2 = 0x00,
	RESERVED3 = 0x00,
	END_BYTE = 0x55
};

//? USB-CAN-A adapter type enum
enum class USBCANFrameType
{
	FIXED = 0x02,
	VARIABLE = 0x12
};
//? USB-CAN-A adapter frame format enum
enum class USBCANFrameFmt
{
	STD = 0x01,
	EXT = 0x02
};
enum class USBCANFrameFmtVar
{
	DATA = 0,
	REMOTE = 1
};
//? USB-CAN-A adapter speed enum
enum class USBCANBaud
{
	SPEED_1000K = 0x01,
	SPEED_800K = 0x02,
	SPEED_500K = 0x03,
	SPEED_400K = 0x04,
	SPEED_250K = 0x05,
	SPEED_200K = 0x06,
	SPEED_125K = 0x07,
	SPEED_100K = 0x08,
	SPEED_50K = 0x09,
	SPEED_20K = 0x0A,
	SPEED_10K = 0x0B,
	SPEED_5K = 0x0C
};

//? USB-CAN-A serial port baud rate enum
enum class USBBaud
{
	BAUD_9600 = 9600,
	BAUD_19200 = 19200,
	BAUD_38400 = 38400,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200,
	BAUD_200000 = 2000000
};

//? USB-CAN-A adapter operating mode enum
enum class USBCANMode
{
	NORMAL = 0x00,
	LOOPBACK = 0x01,
	SILENT = 0x02,
	LOOPBACK_SILENT = 0x03
};

//? USB-CAN-A adapter retransmission enum
enum class USBCANRTX
{
	AUTO = 0x00,
	OFF = 0x01
};

enum class USBCANFrameType {
	STD_VAR,
	EXT_VAR,
	STD_FIXED,
	EXT_FIXED,
	SETTINGS
};

//? defines of default settings
#define USB_DEF_FILTER_SETTING 0x00000000       // Default filter setting (accept all IDs)
#define USB_DEF_MASK_SETTING 0x00000000 // Default mask setting (no bits are
#define USB_DEF_FRAME_FORMAT USBCANFrameType::VARIABLE    // Default frame format for the adapter
#define USB_DEF_FRAME_TYPE USBCANFrameFmt::STD // Default frame type for the adapter
#define USB_DEF_CAN_SPEED USBCANBaud::SPEED_1000K     // Default CAN bus speed
#define USB_DEF_BAUD_RATE USBBaud::BAUD_200000      // Default baud rate for the serial interface
#define USB_DEF_CAN_MODE USBCANMode::NORMAL // Default operating mode for the adapter
#define USB_DEF_RTX USBCANRTX::AUTO // Default auto-retransmission setting