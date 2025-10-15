#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <fstream>
#include <chrono>
#include <mutex>
#include <cmath>

#include "../include/frameManager.hpp"
#include "../include/readStatusWord.hpp"

#define FEED_CONSTANT 65536
#define REDUCTION_FACTOR 36

typedef enum
{
    SDO_EXPEDITED_TRANSFER = 1,
    SD_SEGMENTED_TRANSFER = 0,
} SDO_TRANSFER_TYPE;

typedef enum
{
    INITIATE_DOMAIN_DOWNLOAD = 1,
    DOWNLOAD_DOMAIN_SEGMENT = 2,
    INITIATE_DOMAIN_UPLOAD = 3,
    UPLOAD_DOMAIN_SEGMENT = 4,
    ABORT_DOMAIN_TRANSFER = 5,
} SDO_PROTOCOLS;

typedef enum
{
    EMERGENCY_FC = 0x80,
    TPDO_1_FC = 0x180,
    RPDO_1_FC = 0x200,
    TPDO_2_FC = 0x280,
    RPDO_2_FC = 0x300,
    TPDO_3_FC = 0x380,
    RPDO_3_FC = 0x400,
    TPDO_4_FC = 0x480,
    RPDO_4_FC = 0x500,
    SDO_SERVER_FC = 0x580,
    SDO_CLIENT_FC = 0x600,
} DS_301_FC; // FC : function code

typedef unsigned char DS_301_CS_CLIENT; // CS : command specifier
typedef unsigned char DS_301_CS_SERVER;

DS_301_CS_CLIENT sdo_fc_to_cs_client(SDO_PROTOCOLS fc);

typedef enum
{
    TPDO = 0,
    RPDO = 1,
} PDO_TYPE;

int set_speed(int tty_fd, double speed, int node_id, bool method);

int go_to_operational_enable(int tty_fd, DSP402_STATES current_state);

int set_profile_acceleration(int tty_fd, int profile_acceleration, int profile_deceleration);

int read_velocity_actual_value(int tty_fd, bool method);

int sendSDO(int tty_fd, int node_id, SDO_PROTOCOLS prot, SDO_TRANSFER_TYPE transfer_type, int index, int subindex, unsigned char *data, int data_len);

int mapPDO(int tty_fd, int node_id, PDO_TYPE PDOtype, int PDOnumber, unsigned char data[], int data_len);

int rpm2internal(double rpm, int feed_constant, int reduction_factor);

double internal2rpm(int internal_value, int feed_constant, int reduction_factor);


#endif