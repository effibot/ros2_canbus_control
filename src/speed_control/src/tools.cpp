#include "../include/tools.hpp"

#include <termios.h>

extern std::mutex mtx;

int rpm2internal(double rpm, int feed_constant, int reduction_factor)
{
    return (rpm * feed_constant * reduction_factor) / 60;
}

double internal2rpm(int internal_value, int feed_constant, int reduction_factor)
{
    return (internal_value * 60.0) / (1.0 * feed_constant * reduction_factor);
}

int read_velocity_actual_value(int tty_fd, bool method)
{

    int result;

    long int data = 0x0000000000606c40;

    int data_length_code = sizeof(data) / sizeof(unsigned char);

    // Registra il tempo iniziale
    auto start_time = std::chrono::steady_clock::now();

    auto last_time = start_time;

    unsigned char frame[13] = {0};

    unsigned char recv_data[4];

    int frame_len = 0;

    int cont = 0;
    int max_lim = 100;

    int in_loop = 0;

    int actual_speed = 0;

    while (1)
    {
        frame[13] = {0};

    redo2:

        if (method)
        {
            printf("Speed request: ");

            result = frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&data, data_length_code);

            if (result == -1)
            {

                if (errno == EAGAIN)
                {
                    printf("riprovo\n");
                }

                printf("read_velocity_actual_value: %s\n", strerror(errno));
                goto redo2;
            }
        }

redo3:

        // tcflush(tty_fd, TCOFLUSH);
        printf("Actual speed: ");
        usleep(0.001 * 1e6);
        frame_len = frame_recv(tty_fd, frame, sizeof(frame));

        if (frame_len == -1)
        {
            continue;
        }

        if (frame_len == -1)
        {
            if (errno == EAGAIN)
            {
                printf("riprovo\n");
                goto redo2;
            }
            printf("Frame receive error!\n");
        }
        else
        {

            if ((frame_len >= 6) &&
                (frame[0] == 0xaa) &&
                ((frame[1] >> 4) == 0xc))
            {
                if (method && (frame[6] != 0x60 || frame[5] != 0x6c))
                {
                    //printf("SDO: ricevuto frame di %d byte non idoneo di indice: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", frame_len, frame[13], frame[12], frame[11], frame[10], frame[9], frame[8], frame[7], frame[6], frame[5], frame[4], frame[3], frame[2], frame[1], frame[0]);
                    goto redo3;
                }

                if (!method && (frame[3] != 0x05 || frame[2] != 0x81))
                {

                    //printf("PDO: ricevuto frame di %d byte non idoneo di indice: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", frame_len, frame[13], frame[12], frame[11], frame[10], frame[9], frame[8], frame[7], frame[6], frame[5], frame[4], frame[3], frame[2], frame[1], frame[0]);
                    continue;
                }

                if (method)
                {
                    actual_speed = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24);
                }
                else
                {
                    actual_speed = frame[4] | (frame[5] << 8) | (frame[6] << 16) | (frame[7] << 24);
                }

                // printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x\n", frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7], frame[8]);
            }
        }

        if (~in_loop)
        {
            return actual_speed;
        }
    }

    return 0;
}

int set_profile_acceleration(int tty_fd, int profile_acceleration, int profile_deceleration)
{

    unsigned char frame[13];

    long int profile_acc_hex = profile_acceleration;
    long int profile_dece_hex = profile_deceleration;

    long int data = 0x0000000000608323;

    long int acc_data = data | (profile_acc_hex << 32);

    int data_length_code = sizeof(acc_data) / sizeof(unsigned char);

    frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&acc_data, data_length_code);

    usleep(0.5 * 1e6);

    frame_recv(tty_fd, frame, sizeof(frame));

    printf("<<< ");
    for (int i = 0; i <= sizeof(frame); i++)
    {
        printf("%02x ", frame[i]);
    }
    printf("\n");

    data = 0x0000000000608423;

    long int dece_data = data | (profile_dece_hex << 32);

    frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&dece_data, data_length_code);

    usleep(0.5 * 1e6);

    frame_recv(tty_fd, frame, sizeof(frame));

    printf("<<< ");
    for (int i = 0; i <= sizeof(frame); i++)
    {
        printf("%02x ", frame[i]);
    }
    printf("\n");

    return 0;
}

int go_to_operational_enable(int tty_fd, DSP402_STATES current_state)
{

    int wait_delay = 0.5 * 1e6; // ms

    unsigned char frame[13];

    long int data = 0x000000060060402f;

    int data_length_code = sizeof(data) / sizeof(unsigned char);

    if (current_state != OPERATION_ENABLED)
    {

        frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&data, data_length_code);
        
        usleep(wait_delay);

        frame_recv(tty_fd, frame, sizeof(frame));

        usleep(wait_delay);

        data = 0x000000070060402f;

        frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&data, data_length_code);

        usleep(wait_delay);

        data = 0x000000f0060402f;

        frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&data, data_length_code);
    }
    else
    {
        printf("non è necessario cambiare stato!\n");
    }

    return 0;
}

int set_speed(int tty_fd, double rpm, int node_id, bool method)
{
    int result;

    unsigned char lsb_cob_id, msb_cob_id;

    int speed = rpm2internal(rpm, FEED_CONSTANT, REDUCTION_FACTOR);

    long int speed_hex = speed;

    long int data = 0;

    int data_length_code;

    if (method)
    {
        // SDO
        lsb_cob_id = 0x00 + node_id;
        msb_cob_id = 0x06;
        data = 0x000000000060ff23;

        data |= speed_hex << 32;

        data_length_code = 8;

        // printf("SDO: COB-ID: (%x%02x), DATA: %02lx \n", msb_cob_id, lsb_cob_id, data);
    }
    else
    {
        // PDO
        lsb_cob_id = 0x00 + node_id;
        msb_cob_id = 0x02;

        data = speed;

        data_length_code = 4;

        // printf("PDO: COB-ID: (%x%02x), DATA: %02lx\n", msb_cob_id, lsb_cob_id, data);
    }

redo:

    printf("Command speed: ");
    result = frame_send(tty_fd, CANUSB_FRAME_STANDARD, lsb_cob_id, msb_cob_id, (unsigned char *)&data, data_length_code);

    if (result == -1)
    {

        if (errno == EAGAIN)
        {
            goto redo;
        }

        printf("set_speed failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}