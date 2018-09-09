#ifndef COMMS_H
#define COMMS_H

// Defined here: https://github.com/berkeley-open-robotics/bldc-controller/docs/
// Current Protocol = Version 3

//#define DEBUG_TRANSMIT
//#define DEBUG_RECEIVE
//#define DEBUG_RECEIVE_CRC
//#define DEBUG_CALIBRATION_DATA
//#define DEBUG_ALLOCS

constexpr uint8_t COMM_SYNC_FLAG = 0xFF;
// Reverse index from 0xFF (Protocol V1 and V2 use FF)
using comm_protocol_t = uint8_t;
constexpr comm_protocol_t COMM_VERSION = 0xFE;

using comm_ctrl_mode_t = uint8_t;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_CURRENT = 0x00;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_RAW_PWM = 0x01;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_TORQUE = 0x02;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_VELOCITY = 0x03;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_POSITION = 0x04;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_POS_VEL = 0x05;
constexpr comm_ctrl_mode_t COMM_CTRL_MODE_PWM_DRIVE = 0x06;

// Message lengths are stored at 2 bytes
using comm_msg_len_t = uint16_t;

// Errors Definitions
using comm_errors_t = uint16_t;
constexpr comm_errors_t COMM_ERRORS_NONE = 0;
constexpr comm_errors_t COMM_ERRORS_OP_FAILED = 1;
constexpr comm_errors_t COMM_ERRORS_MALFORMED = 2;
constexpr comm_errors_t COMM_ERRORS_INVALID_FC = 4;
constexpr comm_errors_t COMM_ERRORS_INVALID_ARGS = 8;
constexpr comm_errors_t COMM_ERRORS_BUF_LEN_MISMATCH = 16;

// Board ID Limits
using comm_id_t = uint8_t;
constexpr comm_id_t COMM_ID_BROADCAST = 0;
constexpr comm_id_t COMM_ID_MIN = 1;
constexpr comm_id_t COMM_ID_MAX = UINT8_MAX;

// Flag Bits
using comm_fg_t = uint8_t;
constexpr comm_fg_t COMM_FG_COMP = 0x00;
constexpr comm_fg_t COMM_FG_BOARD = 0x01;

using comm_fc_t = uint8_t;
constexpr comm_fc_t COMM_FC_NOP = 0x00;
constexpr comm_fc_t COMM_FC_REG_READ = 0x01;
constexpr comm_fc_t COMM_FC_REG_WRITE = 0x02;
constexpr comm_fc_t COMM_FC_REG_READ_WRITE = 0x03;
constexpr comm_fc_t COMM_FC_SYSTEM_RESET = 0x80;
constexpr comm_fc_t COMM_FC_JUMP_TO_ADDR = 0x81;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_COUNT = 0x82;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_START = 0x83;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_SIZE = 0x84;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_ERASE = 0x85;
constexpr comm_fc_t COMM_FC_FLASH_PROGRAM = 0x86;
constexpr comm_fc_t COMM_FC_FLASH_READ = 0x87;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY = 0x88;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY_ERASED = 0x89;

using comm_addr_t = uint16_t;
// Calibration Register (0x1***)
constexpr comm_addr_t COMM_REG_CAL_REV_START = 0x1000;
constexpr comm_addr_t COMM_REG_CAL_EREVS_PER_MREV = 0x1001;
constexpr comm_addr_t COMM_REG_CAL_INV_PHASES = 0x1002;
constexpr comm_addr_t COMM_REG_CAL_DI_KP = 0x1003;
constexpr comm_addr_t COMM_REG_CAL_DI_KI = 0x1004;
constexpr comm_addr_t COMM_REG_CAL_QI_KP = 0x1005;
constexpr comm_addr_t COMM_REG_CAL_QI_KI = 0x1006;
constexpr comm_addr_t COMM_REG_CAL_V_KP = 0x1007;
constexpr comm_addr_t COMM_REG_CAL_V_KI = 0x1008;
constexpr comm_addr_t COMM_REG_CAL_P_KP = 0x1009;
constexpr comm_addr_t COMM_REG_CAL_P_KI = 0x100a;
constexpr comm_addr_t COMM_REG_CAL_I_LIMIT = 0x1010;
constexpr comm_addr_t COMM_REG_CAL_T_LIMIT = 0x1011;
constexpr comm_addr_t COMM_REG_CAL_V_LIMIT = 0x1012;
constexpr comm_addr_t COMM_REG_CAL_POS_L_LIMIT = 0x1013;
constexpr comm_addr_t COMM_REG_CAL_POS_U_LIMIT = 0x1014;
constexpr comm_addr_t COMM_REG_CAL_POS_OFFSET = 0x1015;
constexpr comm_addr_t COMM_REG_CAL_MOTOR_R = 0x1020; // Resistance
constexpr comm_addr_t COMM_REG_CAL_MOTOR_H = 0x1021; // Inductance
constexpr comm_addr_t COMM_REG_CAL_MOTOR_T = 0x1022; // Torque Constant
constexpr comm_addr_t COMM_REG_CAL_WATCHDOG = 0x1030;
constexpr comm_addr_t COMM_REG_CAL_V_FILTER = 0x1040;
constexpr comm_addr_t COMM_REG_CAL_EAC_SCALE = 0x1100;
constexpr comm_addr_t COMM_REG_CAL_EAC_OFFSET = 0x1101;
constexpr comm_addr_t COMM_REG_CAL_EAC_TABLE = 0x1200;

// Volatile Registers (0x2***)
constexpr comm_addr_t COMM_REG_VOL_CTRL_MODE = 0x2000;
constexpr comm_addr_t COMM_REG_VOL_DI_COMM = 0x2001;
constexpr comm_addr_t COMM_REG_VOL_QI_COMM = 0x2002;
constexpr comm_addr_t COMM_REG_VOL_PHASE_A_PWM = 0x2003;
constexpr comm_addr_t COMM_REG_VOL_PHASE_B_PWM = 0x2004;
constexpr comm_addr_t COMM_REG_VOL_PHASE_C_PWM = 0x2005;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_T = 0x2006;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_V = 0x2007;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_P = 0x2008;
constexpr comm_addr_t COMM_REG_VOL_PWM_DRIVE = 0x2009;
// Read Only Registers (0x3***)
constexpr comm_addr_t COMM_REG_RO_ROTOR_P = 0x3000;
constexpr comm_addr_t COMM_REG_RO_ROTOT_V = 0x3001;
constexpr comm_addr_t COMM_REG_RO_DI_MEAS = 0x3002;
constexpr comm_addr_t COMM_REG_RO_QC_MEAS = 0x3003;
constexpr comm_addr_t COMM_REG_RO_DC_SUPPLY_V = 0x3004;
constexpr comm_addr_t COMM_REG_RO_TEMP = 0x3005;
constexpr comm_addr_t COMM_REG_RO_ACCEL_X = 0x3006;
constexpr comm_addr_t COMM_REG_RO_ACCEL_Y = 0x3007;
constexpr comm_addr_t COMM_REG_RO_ACCEL_Z = 0x3008;
constexpr comm_addr_t COMM_REG_RO_REC_START = 0x3009;
constexpr comm_addr_t COMM_REG_RO_REC_LEN = 0x300a;
constexpr comm_addr_t COMM_REG_RO_REC_RESET = 0x300b;
constexpr comm_addr_t COMM_REG_RO_ROTOR_P_RAW = 0x300c;

using comm_reg_count_t = uint8_t;

using comm_full_addr_t = uint32_t;
constexpr comm_full_addr_t COMM_BOOTLOADER_OFFSET = 0x08000000;
constexpr comm_full_addr_t COMM_NVPARAMS_OFFSET = 0x08004000;
constexpr comm_full_addr_t COMM_FIRMWARE_OFFSET = 0x08008000;
constexpr comm_full_addr_t COMM_DEFAULT_BAUD_RATE = 1000000;

constexpr size_t COMM_MAX_BUF = 256;
constexpr size_t COMM_MAX_RETRIES = 6;
constexpr size_t COMM_SINGLE_PROGRAM_LENGTH = 128;
constexpr size_t COMM_SINGLE_READ_LENGTH = 128;
constexpr size_t COMM_SINGLE_VERIFY_LENGTH = 128;
constexpr size_t COMM_SINGLE_SET_EAC_TABLE_LENGTH = 64;

#endif /* COMMS_H */
