#ifndef FRAME_MANAGER_HPP
#define FRAME_MANAGER_HPP

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ctype.h>
#include <mutex>

#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000 //2000000
#define MAX_FRAME_SIZE 13

typedef enum
{
    CANUSB_FRAME_STANDARD = 0x01,
    CANUSB_FRAME_EXTENDED = 0x02,
} CANUSB_FRAME;

typedef enum {
  CANUSB_SPEED_1000000 = 0x01,
  CANUSB_SPEED_800000  = 0x02,
  CANUSB_SPEED_500000  = 0x03,
  CANUSB_SPEED_400000  = 0x04,
  CANUSB_SPEED_250000  = 0x05,
  CANUSB_SPEED_200000  = 0x06,
  CANUSB_SPEED_125000  = 0x07,
  CANUSB_SPEED_100000  = 0x08,
  CANUSB_SPEED_50000   = 0x09,
  CANUSB_SPEED_20000   = 0x0a,
  CANUSB_SPEED_10000   = 0x0b,
  CANUSB_SPEED_5000    = 0x0c,
} CANUSB_SPEED;

typedef enum {
  CANUSB_MODE_NORMAL          = 0x00,
  CANUSB_MODE_LOOPBACK        = 0x01,
  CANUSB_MODE_SILENT          = 0x02,
  CANUSB_MODE_LOOPBACK_SILENT = 0x03,
} CANUSB_MODE;


int frame_send(int tty_fd, CANUSB_FRAME frame, unsigned char id_lsb, unsigned char id_msb, unsigned char data[], int data_length_code);

int generate_checksum(const unsigned char *data, int data_len);

int frame_is_complete(const unsigned char *frame, int frame_len);

int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max);

int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame);

int adapter_init(std::string tty_device, int baudrate);

#endif