#include "../include/tools.hpp"

int mapPDO(int tty_fd, int node_id, PDO_TYPE PDOtype, int PDOnumber, unsigned char data[], int data_len)
{
    short PDO_cp_addr; // PDO communication parameter address
    short PDO_mp_addr; // PDO mapping parameter address
    short PDO_cob_id;  // PDO COB-ID (0x200 + node_id for RPDO and 0x180 + node_id for TPD0)

    if (PDOtype == RPDO)
    {
        PDO_cp_addr = 0x1400 + PDOnumber - 1;
        PDO_mp_addr = 0x1600 + PDOnumber - 1;
        PDO_cob_id = 0x200 + 256 * (PDOnumber - 1) + node_id;
    }
    else
    {
        PDO_cp_addr = 0x1800 + PDOnumber - 1;
        PDO_mp_addr = 0x1A00 + PDOnumber - 1;
        PDO_cob_id = 0x180 + 256 * (PDOnumber - 1) + node_id;
    }

    int sdo_data = 0xC0000000 + PDO_cob_id;
    // Set COB-ID of PDO and disabled it
    sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_cp_addr, 0x01, (unsigned char *)&sdo_data, 4);
    usleep(0.1 * 1e6);

    sdo_data = 0x000000FF;
    // Set Transmission type of PDO to 0xff
    sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_cp_addr, 0x02, (unsigned char *)&sdo_data, 1);
    usleep(0.1 * 1e6);

    sdo_data = 0x000007D0;
    // Set Inhibit time 
    sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_cp_addr, 0x03, (unsigned char *)&sdo_data, 2);
    usleep(0.1 * 1e6);

    if (PDOtype == TPDO)
    {
        sdo_data = 0x000001F4;
        // Set Timer of PDO to 25ms (0xFA)
        sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_cp_addr, 0x05, (unsigned char *)&sdo_data, 2);
        usleep(0.1 * 1e6);
    }

    sdo_data = 0x00000000;
    // Set that there are 0 object mapped in PDO
    sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_mp_addr, 0x00, (unsigned char *)&sdo_data, 1);
    usleep(0.1 * 1e6);

    int n = data_len;
    unsigned char entry[4];
    unsigned char lsb_index, msb_index, subindex;
    unsigned char pdo_data_len;

    // Map object with index and sub-index, dimension in PDO
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            entry[j] = (data[i + j]) & 0xFF;
        }

        lsb_index = entry[3];
        msb_index = entry[2];
        subindex = entry[1];
        pdo_data_len = entry[0];

        sdo_data = lsb_index << 24 | msb_index << 16 | subindex << 8 | pdo_data_len;

        sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_mp_addr, i + 1, (unsigned char *)&sdo_data, 4);

        usleep(0.1 * 1e6);
    }

    sdo_data = n;

    // Set that there are n object mapped in PDO
    sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_mp_addr, 0x00, (unsigned char *)&sdo_data, 1);

    usleep(0.1 * 1e6);

    sdo_data = 0x40000000 + PDO_cob_id;

    // Set COB-ID of PDO and enabled
    sendSDO(tty_fd, node_id, INITIATE_DOMAIN_DOWNLOAD, SDO_EXPEDITED_TRANSFER, PDO_cp_addr, 0x01, (unsigned char *)&sdo_data, 4);

    usleep(0.1 * 1e6);

    return 0;
}