#include "../include/frameManager.hpp"
#include <asm/termbits.h> /* struct termios2 */

std::mutex mtx; // Declare a mutex

int adapter_init(std::string tty_device, int baudrate)
{
  int tty_fd, result;
  struct termios2 tio;

  tty_fd = open(tty_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  usleep(10000);

  if (tty_fd == -1)
  {
    fprintf(stderr, "open(%s) failed: %s\n", tty_device, strerror(errno));
    return -1;
  }

  result = ioctl(tty_fd, TCGETS2, &tio);

  if (result == -1)
  {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  // Serial port configuration
  tio.c_cflag &= ~CBAUD;               // Rimozione del flag CBAUD
  tio.c_cflag  = BOTHER | CS8 | CSTOPB; // Aggiunta dei flag BOTHER, CS8 e CSTOPB
  tio.c_iflag  = IGNPAR;                // Si ignorano gli errori relativi al parity bit
  tio.c_oflag  = 0;                     // Disabilita le opzioni di output (nessuna elaborazione).
  tio.c_lflag  = 0;                     // Disabilita la modalità canonica e i segnali
  tio.c_ispeed = baudrate;             // Impostazione velocità di trasmissione input
  tio.c_ospeed = baudrate;             // Impostazione velocità di trasmissione output

  result = ioctl(tty_fd, TCSETS2, &tio);

  if (result == -1)
  {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  return tty_fd;
}

int frame_send(int tty_fd, CANUSB_FRAME frame, unsigned char id_lsb, unsigned char id_msb, unsigned char data[], int data_length_code)
{

  int data_frame_len = 0, result = 0;
  unsigned char data_frame[MAX_FRAME_SIZE] = {0x00};

  if (data_length_code < 0 || data_length_code > 8)
  {
    fprintf(stderr, "Data length code (DLC) must be between 0 and 8!\n");
    return -1;
  }

  /* Byte 0: Packet Start */
  data_frame[data_frame_len++] = 0xaa;

  /* Byte 1: CAN Bus Data Frame Information */
  data_frame[data_frame_len] = 0x00;
  data_frame[data_frame_len] |= 0xC0; /* Bit 7 Always 1, Bit 6 Always 1 */
  if (frame == CANUSB_FRAME_STANDARD)
    data_frame[data_frame_len] &= 0xDF;           /* STD frame */
  else                                            /* CANUSB_FRAME_EXTENDED */
    data_frame[data_frame_len] |= 0x20;           /* EXT frame */
  data_frame[data_frame_len] &= 0xEF;             /* 0=Data */
  data_frame[data_frame_len] |= data_length_code; /* DLC=data_len */
  data_frame_len++;

  /* Byte 2 to 3: ID */
  data_frame[data_frame_len++] = id_lsb; /* lsb */
  data_frame[data_frame_len++] = id_msb; /* msb */

  /* Byte 4 to (4+data_len): Data */
  for (int i = 0; i < data_length_code; i++)
    data_frame[data_frame_len++] = data[i];

  /* Last byte: End of frame */
  data_frame[data_frame_len++] = 0x55;

  result = write(tty_fd, data_frame, data_frame_len);

  if (result == -1)
  {
    // fprintf(stderr, "write() failed: %s\n", strerror(errno));
    return -1;
  }

  printf(">>> ");
  for (int i = 0; i < data_frame_len; i++)
  {
       printf("%02x ", data_frame[i]);
   }
   printf("\n");

  return 0;
}

int generate_checksum(const unsigned char *data, int data_len)
{
  int i, checksum;

  checksum = 0;
  for (i = 0; i < data_len; i++)
  {
    checksum += data[i];
  }

  return checksum & 0xff;
}

int frame_is_complete(const unsigned char *frame, int frame_len)
{
  if (frame_len > 0)
  {
    if (frame[0] != 0xaa)
    {
      /* Need to sync on 0xaa at start of frames, so just skip. */
      return 1;
    }
  }

  if (frame_len < 2)
  {
    return 0;
  }

  if (frame[1] == 0x55)
  { /* Command frame... */
    if (frame_len >= 20)
    { /* ...always 20 bytes. */
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else if ((frame[1] >> 4) == 0xc)
  { /* Data frame... */
    if (frame_len >= (frame[1] & 0xf) + 5)
    { /* ...payload and 5 bytes. */
      return 1;
    }
    else
    {
      return 0;
    }
  }

  /* Unhandled frame type. */
  return 1;
}

int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max)
{
  int result, frame_len, checksum;
  unsigned char byte;

  // if (print_traffic)
  printf("<<< ");

  frame_len = 0;
  while (1)
  {

  redo:

    result = read(tty_fd, &byte, 1);

    if (result == -1)
    {
      if (errno != EAGAIN && errno != EWOULDBLOCK)
      {
        fprintf(stderr, "read() failed: %s\n", strerror(errno));
        return -1;
      }
      if (errno == EAGAIN) {
        goto redo;
      }
    }
    else if (result > 0)
    {
      // if (print_traffic)
      printf("%02x ", byte);

      if (frame_len == frame_len_max)
      {
        fprintf(stderr, "frame_recv() failed: Overflow\n");
        return -1;
      }

      frame[frame_len++] = byte;

      if ((byte == 0x55) && (frame_len > 2))
      {
        // printf("breakpoint %02x letto con %d\n",byte,frame_len);
        break;
      }

      if (frame_is_complete(frame, frame_len))
      {
        break;
      }
    }

    usleep(10);
  }

  printf("\n");

  /* Compare checksum for command frames only. */
  if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55))
  {
    checksum = generate_checksum(&frame[2], 17);
    if (checksum != frame[frame_len - 1])
    {
      fprintf(stderr, "frame_recv() failed: Checksum incorrect\n");
      return -1;
    }
  }

  return frame_len;
}

int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame)
{
  int cmd_frame_len, result;
  unsigned char cmd_frame[20];

  cmd_frame_len = 0;
  cmd_frame[cmd_frame_len++] = 0xaa;
  cmd_frame[cmd_frame_len++] = 0x55;
  cmd_frame[cmd_frame_len++] = 0x12;
  cmd_frame[cmd_frame_len++] = speed;
  cmd_frame[cmd_frame_len++] = frame;
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = mode;
  cmd_frame[cmd_frame_len++] = 0x01;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = generate_checksum(&cmd_frame[2], 17);

  result = write(tty_fd, (const void *)cmd_frame, cmd_frame_len);
  if (result == -1)
  {
    fprintf(stderr, "write() failed: %s\n", strerror(errno));
    return -1;
  }

  return 0;
}