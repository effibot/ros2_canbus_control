#include "../include/tools.hpp"

DS_301_CS_CLIENT sdo_fc_to_cs_client(SDO_PROTOCOLS fc)
{
    switch (fc)
    {
    case INITIATE_DOMAIN_DOWNLOAD:
        return 0x20;
    case DOWNLOAD_DOMAIN_SEGMENT:
        return 0x00;
    case INITIATE_DOMAIN_UPLOAD:
        return 0x40;
    case UPLOAD_DOMAIN_SEGMENT:
        return 0x60;
    case ABORT_DOMAIN_TRANSFER:
        return 0x80;
    default:
        return 0x80;
    }
}

int sendSDO(int tty_fd, int node_id, SDO_PROTOCOLS prot, SDO_TRANSFER_TYPE transfer_type, int index, int subindex, unsigned char *data, int data_len)
{
    short cob_id;
    unsigned char lsb_cob_id, msb_cob_id, lsb_index, msb_index;
    unsigned char n = 0;
    unsigned char cs;
    bool e = 0, s = 0;
    unsigned char data_frame[8] = {0};

    cob_id = SDO_CLIENT_FC + node_id; // COB-ID is given by the sum between function code and the destination node ID
    lsb_cob_id = cob_id & 0xFF;
    msb_cob_id = (cob_id & 0xFF00) >> 8;

    lsb_index = index & 0xFF;
    msb_index = (index & 0xFF00) >> 8; 

    if (prot == INITIATE_DOMAIN_DOWNLOAD)
    {
        n = 4 - data_len; // get the number of bytes that do not contain data
        e = (transfer_type == SDO_EXPEDITED_TRANSFER) ? 1 : 0;
        s = 1;
    }

    cs = sdo_fc_to_cs_client(prot) | (n << 2) | (e << 1) | s;

    data_frame[0] = cs;
    data_frame[1] = lsb_index;
    data_frame[2] = msb_index;
    data_frame[3] = subindex;

    for (int i = 0; i < data_len; i++)
    {
        data_frame[i + 4] = data[i];
    }

    printf("COB-ID: (%x%02x), DATA: %02x %02x %02x %02x %02x %02x %02x %02x (len: %d)\n", msb_cob_id, lsb_cob_id, data_frame[0], data_frame[1], data_frame[2], data_frame[3], data_frame[4], data_frame[5], data_frame[6], data_frame[7], data_len);

    frame_send(tty_fd, CANUSB_FRAME_STANDARD, lsb_cob_id, msb_cob_id, data_frame, 8);

    return 0;
}