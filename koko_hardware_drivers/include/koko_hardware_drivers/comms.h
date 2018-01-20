#ifndef COMMS_H
#define COMMS_H

typedef uint16_t comm_errors_t;
const comm_errors_t COMM_ERRORS_NONE = 0;
const comm_errors_t COMM_ERRORS_OP_FAILED = 1;
const comm_errors_t COMM_ERRORS_MALFORMED = 2;
const comm_errors_t COMM_ERRORS_INVALID_FC = 4;
const comm_errors_t COMM_ERRORS_INVALID_ARGS = 8;
const comm_errors_t COMM_ERRORS_BUF_LEN_MISMATCH = 16;

typedef uint8_t comm_id_t;
const comm_id_t COMM_ID_BROADCAST = 0;
const comm_id_t COMM_ID_MIN = 1;
const comm_id_t COMM_ID_MAX = UINT8_MAX;

typedef uint8_t comm_fc_t;
const comm_fc_t COMM_FC_NOP = 0x00;
const comm_fc_t COMM_FC_REG_READ = 0x01;
const comm_fc_t COMM_FC_REG_WRITE = 0x02;
const comm_fc_t COMM_FC_REG_READ_WRITE = 0x03;
const comm_fc_t COMM_FC_SYSTEM_RESET = 0x80;
const comm_fc_t COMM_FC_JUMP_TO_ADDR = 0x81;
const comm_fc_t COMM_FC_FLASH_SECTOR_COUNT = 0x82;
const comm_fc_t COMM_FC_FLASH_SECTOR_START = 0x83;
const comm_fc_t COMM_FC_FLASH_SECTOR_SIZE = 0x84;
const comm_fc_t COMM_FC_FLASH_SECTOR_ERASE = 0x85;
const comm_fc_t COMM_FC_FLASH_PROGRAM = 0x86;
const comm_fc_t COMM_FC_FLASH_READ = 0x87;
const comm_fc_t COMM_FC_FLASH_VERIFY = 0x88;
const comm_fc_t COMM_FC_FLASH_VERIFY_ERASED = 0x89;

typedef uint16_t comm_addr_t;

typedef uint8_t comm_reg_count_t;

unsigned const int COMM_BOOTLOADER_OFFSET = 0x08000000;
unsigned const int COMM_NVPARAMS_OFFSET = 0x08004000;
unsigned const int COMM_FIRMWARE_OFFSET = 0x08008000;
unsigned const int COMM_DEFAULT_BAUD_RATE = 1000000;

size_t COMM_SINGLE_PROGRAM_LENGTH = 128;
size_t COMM_SINGLE_READ_LENGTH = 128;
size_t COMM_SINGLE_VERIFY_LENGTH = 128;
#endif /* COMMS_H */
