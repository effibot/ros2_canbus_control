#include "../include/readStatusWord.hpp"

bool readBit(short word, int bitPosition) {
    return (word >> bitPosition) & 1; 
}

DSP402_STATES request_status_word(int tty_fd)
{

    int result;

    long int data = 0x0000000000604140;

    int data_length_code = sizeof(data) / sizeof(unsigned char);

    int frame_len = 0;

redo3:

    unsigned char frame[32];

    unsigned char recv_data[4];

    result = frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x01, (unsigned char)0x06, (unsigned char *)&data, data_length_code);

    if (result == -1)
    {
        if (errno == EAGAIN)
        {
            printf("riprovo");
            goto redo3;
        }

        printf("request status word failed: %s", strerror(errno));
    }

    printf("richiesta trasmessa\n");

redo4:

    frame_len = frame_recv(tty_fd, frame, sizeof(frame));

    if (frame_len == -1)
    {
        if (errno == EAGAIN)
        {
            printf("riprovo\n");
            goto redo4;
        }
        printf("Frame receive error!\n");
    }
    else if (frame_len == 0)
    {
        printf("non ho letto nulla\n");
    }
    else
    {
        printf("<<< ");
        for (int i = 0; i <= frame_len; i++)
        {
            printf("%02x ", frame[i]);
        }
        printf("\n");

        if ((frame_len >= 6) &&
            (frame[0] == 0xaa) &&
            ((frame[1] >> 4) == 0xc))
        {
            printf("Frame ID: %02x%02x, Data: ", frame[3], frame[2]);
            int j = 0;
            for (int i = 8; i < frame_len; ++i)
            {
                recv_data[j] = frame[i];
                printf("%02x ", frame[i]);

                ++j;
            }
            printf("\n");
        }
        else
        {
            printf("Unknown: ");
            for (int i = 0; i <= frame_len; i++)
            {
                printf("%02x ", frame[i]);
            }
            printf("\n");
        }
    }

    //short status_word = 0;
    //status_word = recv_data[8] | (recv_data[9] << 8);

    DSP402_STATES current_state;

    current_state = read_status_word(recv_data[0] | (recv_data[1] << 8), VELOCITY_MODE, 0); // Chiamata a functionA

    return current_state;
}

DSP402_STATES read_status_word(short word, DSP402_MODES mode, bool halt)
{
    /*
     *      Read status word (object 0x6041)
     *
     *      @param word 16bit word containing the status word read from the frame
     *      @param mode current mode in object 0x6060
     *      @param halt boolean value which indicates the bit 8 of control word (object 0x4060)
     */

    char state = 0; // we need at least 6 bit so we use a char type
    bool bit10 = 0;

    state |= word & 0xF; // Read first 4 bit of the word

    state |= readBit(word, 6) << 6; // Read the bit 6 of word and set the bit 6 of state

    if (readBit(word, 3) == 0 && readBit(word, 0) == 1)
    {
        state |= readBit(word, 5) << 5;
    }

    switch (state)
    {
    case NOT_READY_TO_SWITCH_ON:
        printf("State: not ready to switch on\n");
        break;
    case SWITCH_ON_DISABLED:
        printf("State: switch on disabled\n");
        break;
    case READY_TO_SWITCH_ON:
        printf("State: ready to switch on\n");
        break;
    case SWITCHED_ON:
        printf("State: switched on\n");
        break;
    case OPERATION_ENABLED:
        printf("State: operation enabled\n");
        break;
    case QUICK_STOP_ACTIVE:
        printf("State: quick stop active\n");
        break;
    case FAULT_REACTION_ACTIVE:
        printf("State: fault reaction active\n");
        break;
    case FAULT:
        printf("State: fault\n");
        break;
    default:
        printf("ERROR: Unabled to identify current state %d!\n", state);
        break;
    }

    if (readBit(word, 4) == 1)
    {
        printf("- High voltage is applied\n");
    }

    if (readBit(word, 7) == 1)
    {
        printf("- There is a drive warning (check 0x3204 and 0x3205)\n");
    }

    if (readBit(word, 9) == 1)
    {
        printf("- Parameters can be modified via the CAN-network\n");
    }
    else
    {
        printf("- Drive is in local mode (no command message)\n");
    }

    if (readBit(word, 11) == 1)
    {
        printf("- Internal limitation is active");
    }

    switch (mode)
    {
    case POSITION_MODE:
        printf("Mode: postion mode\n");

        if (bit10)
        {
            (halt == 1) ? printf("- (HALT) Axle speed = 0\n") : printf("- Target position reached\n");
        }
        else
        {
            (halt == 1) ? printf("- (HALT) Axle decelerates\n") : printf("- Target position NOT reached\n");
        }

        (readBit(word, 12) == 1) ? printf("- Set-point acknowledged\n") : printf("- Set-point NOT acknowledged\n");
        if (readBit(word, 13) == 1)
            printf("- Following error!\n");

        break;
    case VELOCITY_MODE:
        printf("Mode: velocity mode\n");

        if (bit10)
        {
            (halt == 1) ? printf("- (HALT) Axle speed = 0\n") : printf("- Target velocity reached\n");
        }
        else
        {
            (halt == 1) ? printf("- (HALT) Axle decelerates\n") : printf("- Target velocity NOT YET reached\n");
        }

        (readBit(word, 12) == 1) ? printf("- Speed = 0\n") : printf("- Speed != 0\n");
        if (readBit(word, 13) == 1)
            printf("- Maximum slippage reached\n");

        break;
    case HOMING_MODE:
        printf("Mode: homing mode\n");

        if (bit10)
        {
            (halt == 1) ? printf("- (HALT) Axle speed = 0\n") : printf("- Home position reached\n");
        }
        else
        {
            (halt == 1) ? printf("- (HALT) Axle decelerates\n") : printf("- Home position NOT reached\n");
        }

        (readBit(word, 12) == 1) ? printf("- Homing carried out successfully\n") : printf("- Homing not yet completed\n");
        if (readBit(word, 13) == 1)
            printf("- Homing error! (check 0x3204 and 0x3205)\n");

        break;
    case INTERPOLATED_POSITION_MODE:
        printf("Mode: interpolated position mode\n");
        break;
    default:
        printf("ERROR: input mode is not valid");
        break;
    }

    // lettura bit 12 e 13 dipendenti dalla modalita di lavoro

    return (DSP402_STATES)state;
}