import struct
import json
import time

class ProtocolError(Exception):
    def __init__(self, message, errors=None):
        super(ProtocolError, self).__init__(message)

        self.errors = errors

COMM_ERRORS_NONE = 0
COMM_ERRORS_OP_FAILED = 1
COMM_ERRORS_MALFORMED = 2
COMM_ERRORS_INVALID_FC = 4
COMM_ERRORS_INVALID_ARGS = 8
COMM_ERRORS_BUF_LEN_MISMATCH = 16

COMM_FC_NOP = 0x00
COMM_FC_REG_READ = 0x01
COMM_FC_REG_WRITE = 0x02
COMM_FC_REG_READ_WRITE = 0x03
COMM_FC_SYSTEM_RESET = 0x80
COMM_FC_JUMP_TO_ADDR = 0x81
COMM_FC_FLASH_SECTOR_COUNT = 0x82
COMM_FC_FLASH_SECTOR_START = 0x83
COMM_FC_FLASH_SECTOR_SIZE = 0x84
COMM_FC_FLASH_SECTOR_ERASE = 0x85
COMM_FC_FLASH_PROGRAM = 0x86
COMM_FC_FLASH_READ = 0x87
COMM_FC_FLASH_VERIFY = 0x88
COMM_FC_FLASH_VERIFY_ERASED = 0x89

COMM_BOOTLOADER_OFFSET = 0x08000000
COMM_NVPARAMS_OFFSET = 0x08004000
COMM_FIRMWARE_OFFSET = 0x08008000
COMM_DEFAULT_BAUD_RATE = 1000000

COMM_SINGLE_PROGRAM_LENGTH = 128
COMM_SINGLE_READ_LENGTH = 128
COMM_SINGLE_VERIFY_LENGTH = 128

class FlashSectorMap:
    def __init__(self, sector_count, sector_starts, sector_sizes):
        if len(sector_starts) != sector_count or len(sector_sizes) != sector_count:
            raise ValueError('sector_starts and sector_sizes must have the correct length')
        self._sector_count = sector_count
        self._sector_starts = sector_starts
        self._sector_sizes = sector_sizes

    def getFlashSectorCount(self):
        return self._sector_count

    def getFlashSectorStart(self, sector_num):
        return self._sector_starts[sector_num]

    def getFlashSectorEnd(self, sector_num):
        return self._sector_starts[sector_num] + self._sector_sizes[sector_num]

    def getFlashSectorSize(self, sector_num):
        return self._sector_sizes[sector_num]

    def getFlashSectorOfAddress(self, addr):
        for i in range(self.getFlashSectorCount()):
            if self.getFlashSectorStart(i) <= addr < self.getFlashSectorEnd(i):
                return i

        return None

    def getFlashSectorsOfAddressRange(self, addr, length):
        sector_nums = []
        offset = 0

        while offset < length:
            sector_num = self.getFlashSectorOfAddress(addr + offset)

            if sector_num is None:
                return None

            sector_nums.append(sector_num)
            offset = self.getFlashSectorEnd(sector_num) - addr

        return sector_nums

    def __str__(self):
        lines = ['num       start     size']
        for sector_num in range(self.getFlashSectorCount()):
            lines.append('{:3d}  0x{:08x} {:8d}'.format(sector_num, self.getFlashSectorStart(sector_num), self.getFlashSectorSize(sector_num)))
        return '\n'.join(lines)

class BLDCControllerClient:
    def __init__(self, ser):
        self._ser = ser

    def getRotorPosition(self, server_id):
        angle = struct.unpack('<f', self.readRegisters(server_id, 0x3000, 1))[0]
        return angle

    def getState(self, server_id):
        # order: angle, velocity, direct_current, quadrature_current, supply_voltage, board_temp, accel_x, accel_y, accel_z
        state = struct.unpack('<ffffffiii', self.readRegisters(server_id, 0x3000, 9))
        return state

    def getVoltage(self, server_id):
        # order: angle, velocity, direct_current, quadrature_current, supply_voltage, board_temp, accel_x, accel_y, accel_z
        state = struct.unpack('<f', self.readRegisters(server_id, 0x3004, 1))[0]
        return state

    def getTemperature(self, server_id):
        # order: angle, velocity, direct_current, quadrature_current, supply_voltage, board_temp, accel_x, accel_y, accel_z
        state = struct.unpack('<f', self.readRegisters(server_id, 0x3005, 1))[0]
        return state

    def setZeroAngle(self, server_id, value):
        return self.writeRegisters(server_id, 0x1000, 1, struct.pack('<H', value))

    def setInvertPhases(self, server_id, value):
        return self.writeRegisters(server_id, 0x1002, 1, struct.pack('<B', value))

    def setERevsPerMRev(self, server_id, value):
        return self.writeRegisters(server_id, 0x1001, 1, struct.pack('<B', value))

    def setTorqueConstant(self, server_id, value):
        return self.writeRegisters(server_id, 0x1022, 1, struct.pack('<f', value))

    def setPositionOffset(self, server_id, value):
        return self.writeRegisters(server_id, 0x1015, 1, struct.pack('<f', value))

    def setCurrentControlMode(self, server_id):
        return self.writeRegisters(server_id, 0x2000, 1, struct.pack('<B', 0))

    def setCommand(self, server_id, value):
        ret = self.writeRegisters(server_id, 0x2002, 1, struct.pack('<f', value))
        return ret

    def setCommandAndGetState(self, server_id, value):
        ret = self.readWriteRegisters(server_id, 0x3000, 9, 0x2002, 1, struct.pack('<f', value))
        state = struct.unpack('<ffffffiii', ret)
        return state

    def leaveBootloader(self, server_id):
        self.jumpToAddress(server_id, COMM_FIRMWARE_OFFSET)

    def enterBootloader(self, server_id):
        self.resetSystem(server_id)

    def readRegisters(self, server_id, start_addr, count):
        success, data = self.doTransaction(server_id, COMM_FC_REG_READ, struct.pack('<HB', start_addr, count))
        if not success:
            raise IOError("Register read failed %d" % server_id)
        return data

    def writeRegisters(self, server_id, start_addr, count, data):
        success, _ = self.doTransaction(server_id, COMM_FC_REG_WRITE, struct.pack('<HB', start_addr, count) + data)
        return success

    def readWriteRegisters(self, server_id, read_start_addr, read_count, write_start_addr, write_count, write_data):
        message = struct.pack('<HBHB', read_start_addr, read_count, write_start_addr, write_count) + write_data
        success, data = self.doTransaction(server_id, COMM_FC_REG_READ_WRITE, message)
        return data

    def resetSystem(self, server_id):
        self.writeRequest(server_id, COMM_FC_SYSTEM_RESET)
        return True

    def jumpToAddress(self, server_id, jump_addr=COMM_FIRMWARE_OFFSET):
        self.writeRequest(server_id, COMM_FC_JUMP_TO_ADDR, struct.pack('<I', jump_addr))
        return True

    def getFlashSectorCount(self, server_id):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_COUNT, '')
        return struct.unpack('<I', data)[0]

    def getFlashSectorStart(self, server_id, sector_num):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_START, struct.pack('<I', sector_num))
        return struct.unpack('<I', data)[0]

    def getFlashSectorSize(self, server_id, sector_num):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_SIZE, struct.pack('<I', sector_num))
        return struct.unpack('<I', data)[0]

    def eraseFlashSector(self, server_id, sector_num):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_ERASE, struct.pack('<I', sector_num))
        return success

    def programFlash(self, server_id, dest_addr, data):
        for i in range(0, len(data), COMM_SINGLE_PROGRAM_LENGTH):
            success = self._programFlashLimitedLength(server_id, dest_addr + i, data[i:i+COMM_SINGLE_PROGRAM_LENGTH])
            if not success:
                return False
        return True

    def _programFlashLimitedLength(self, server_id, dest_addr, data):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_PROGRAM, struct.pack('<I', dest_addr) + data)
        return success

    def readFlash(self, server_id, src_addr, length):
        data = ''
        for i in range(0, length, COMM_SINGLE_READ_LENGTH):
            read_len = min(length - i, COMM_SINGLE_READ_LENGTH)
            data_chunk = self._readFlashLimitedLength(server_id, src_addr + i, read_len)
            if len(data_chunk) != read_len:
                return False
            data += data_chunk
        return data

    def _readFlashLimitedLength(self, server_id, src_addr, length):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_READ, struct.pack('<II', src_addr, length))
        return data

    def readCalibration(self, server_id):
        l = struct.unpack('<H', self.readFlash(server_id, COMM_NVPARAMS_OFFSET+1, 2))[0]
        return json.loads(self.readFlash(server_id, COMM_NVPARAMS_OFFSET+3, l))

    def verifyFlash(self, server_id, dest_addr, data):
        for i in range(0, len(data), COMM_SINGLE_VERIFY_LENGTH):
            success = self._verifyFlashLimitedLength(server_id, dest_addr + i, data[i:i+COMM_SINGLE_VERIFY_LENGTH])
            if not success:
                return False
        return True

    def _verifyFlashLimitedLength(self, server_id, dest_addr, data):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_VERIFY, struct.pack('<I', dest_addr) + data)
        return success

    def verifyFlashErased(self, server_id, dest_addr, length):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_VERIFY_ERASED, struct.pack('<II', dest_addr, length))
        return success

    def eraseFlash(self, server_id, addr, length, sector_map=None):
        if sector_map is None:
            sector_map = self.getFlashSectorMap(server_id)

        # Find out which sectors need to be erased
        sector_nums = sector_map.getFlashSectorsOfAddressRange(addr, length)

        for sector_num in sector_nums:
            success = self.eraseFlashSector(server_id, sector_num)
            if not success:
                return False

        return True

    def writeFlash(self, server_id, dest_addr, data, sector_map=None, print_progress=False):
        if sector_map is None:
            sector_map = self.getFlashSectorMap(server_id)

        if print_progress:
            print "Erasing flash"

        success = self.eraseFlash(server_id, dest_addr, len(data), sector_map)
        if not success:
            return False

        if print_progress:
            print "Verifying flash was erased"

        success = self.verifyFlashErased(server_id, dest_addr, len(data))
        if not success:
            return False

        if print_progress:
            print "Programming flash"

        success = self.programFlash(server_id, dest_addr, data)
        if not success:
            return False

        if print_progress:
            print "Verifying flash was programmed"

        success = self.verifyFlash(server_id, dest_addr, data)
        if not success:
            return False

        return True

    def getFlashSectorMap(self, server_id):
        sector_count = self.getFlashSectorCount(server_id)
        sector_starts = []
        sector_sizes = []

        for sector_num in range(sector_count):
            sector_starts.append(self.getFlashSectorStart(server_id, sector_num))
            sector_sizes.append(self.getFlashSectorSize(server_id, sector_num))

        return FlashSectorMap(sector_count, sector_starts, sector_sizes)

    def doTransaction(self, server_id, func_code, data):
        self.writeRequest(server_id, func_code, data)
        return self.readResponse(server_id, func_code)

    def writeRequest(self, server_id, func_code, data=''):
        message = struct.pack('BB', server_id, func_code) + data
        prefixed_message = struct.pack('BBH', 0xFF, 0xFF, len(message)) + message
        datagram = prefixed_message + struct.pack('<H', self._computeCRC(prefixed_message))

        self._ser.write(datagram)

    def readResponse(self, server_id, func_code):
        sync = self._ser.read()
        if len(sync) != 1 or sync != "\xff":
            # Reached maximum number of tries
            # self._ser.flushInput()
            return False, None

        version = self._ser.read()
        if len(version) != 1 or version != "\xff":
            # self._ser.flushInput()
            return False, None

        length = self._ser.read(2)
        if length == None or len(length) == 0:
            return False, None

        message_len, = struct.unpack('H', length)
        message = self._ser.read(message_len)

        if len(message) < message_len:
            # self._ser.flushInput()
            return False, None

        crc_bytes = self._ser.read(2)

        if len(crc_bytes) < 2:
            # self._ser.flushInput()
            return False, None

        message_server_id, message_func_code, errors = struct.unpack('<BBH', message[:4])

        if message_server_id != server_id:
            raise ProtocolError('received unexpected server ID: saw ' \
                                + str(message_server_id) + ', expected ' + str(server_id))

        if message_func_code != func_code:
            raise ProtocolError('received unexpected func ID: saw ' \
                                + str(message_func_code) + ', expected ' + str(func_code))

        message_crc, = struct.unpack('<H', crc_bytes)

        if message_crc != self._computeCRC(length + message):
            raise ProtocolError('received unexpected CRC')

        success = (errors & COMM_ERRORS_OP_FAILED) == 0

        # Raise an exception if another type of error occurred
        if (errors & ~COMM_ERRORS_OP_FAILED) != 0:
            raise ProtocolError('other error flags set', errors)

        # if (errors & COMM_ERRORS_OP_FAILED) != 0:
        #     raise ProtocolError('operation failed')

        # if (errors & COMM_ERRORS_MALFORMED) != 0:
        #     raise ProtocolError('malformed request')

        # if (errors & COMM_ERRORS_INVALID_FC) != 0:
        #     raise ProtocolError('invalid function code')

        # if (errors & COMM_ERRORS_INVALID_ARGS) != 0:
        #     raise ProtocolError('invalid arguments')

        # if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) != 0:
        #     raise ProtocolError('buffer length mismatch')

        return success, message[4:]

    def _computeCRC(self, values):
        return 0 # TODO
