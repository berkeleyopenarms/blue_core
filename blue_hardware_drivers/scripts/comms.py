import struct
import json
import time
import crcmod

class ProtocolError(Exception):
    def __init__(self, message, errors=None):
        super(ProtocolError, self).__init__(message)

        self.errors = errors

COMM_VERSION = 0xFE

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

COMM_FLAG_SEND = 0x00 

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

    def getRotorPosition(self, server_ids):
        angles = [struct.unpack('<f', data)[0] for data in self.readRegisters(server_ids, [0x3000 for sid in server_ids], [1 for sid in server_ids])]
        return angles

    def getState(self, server_ids):
        # order: angle, velocity, direct_current, quadrature_current, supply_voltage, board_temp, accel_x, accel_y, accel_z
        states = [struct.unpack('<ffffffiii', data) for data in self.readRegisters(server_ids, [0x3000 for sid in server_ids], [9 for sid in server_ids])]
        return states

    def getVoltage(self, server_ids):
        # order: angle, velocity, direct_current, quadrature_current, supply_voltage, board_temp, accel_x, accel_y, accel_z
        states = [struct.unpack('<f', data)[0] for data in self.readRegisters(server_ids, [0x3004 for sid in server_ids], [1 for sid in server_ids])]
        return states

    def getTemperature(self, server_ids):
        # order: angle, velocity, direct_current, quadrature_current, supply_voltage, board_temp, accel_x, accel_y, accel_z
        states = [struct.unpack('<f', data)[0] for data in self.readRegisters(server_ids, [0x3005 for sid in server_ids], [1 for sid in server_ids])]
        return states

    def setZeroAngle(self, server_ids, value):
        return self.writeRegisters(server_ids, [0x1000 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<H', val) for val in value])

    def setInvertPhases(self, server_ids, value):
        return self.writeRegisters(server_ids, [0x1002 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<B', val) for val in value])

    def setERevsPerMRev(self, server_ids, value):
        return self.writeRegisters(server_ids, [0x1001 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<B', val) for val in value])

    def setTorqueConstant(self, server_ids, value):
        return self.writeRegisters(server_ids, [0x1022 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<f', val) for val in value])

    def setPositionOffset(self, server_ids, value):
        return self.writeRegisters(server_ids, [0x1015 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<f', val) for val in value])

    def setCurrentControlMode(self, server_ids):
        return self.writeRegisters(server_ids, [0x2000 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<B', 0) for sid in server_ids])

    def setCommand(self, server_ids, value):
        ret = self.writeRegisters(server_ids, [0x2002 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<f', val) for val in value])
        return ret

    def setCommandAndGetState(self, server_ids, value):
        ret = self.readWriteRegisters(server_ids, [0x3000 for sid in server_ids], [9 for sid in server_ids], [0x2002 for sid in server_ids], [1 for sid in server_ids], [struct.pack('<f', val) for val in value])
        states = [struct.unpack('<ffffffiii', data) for data in ret]
        return states

    def leaveBootloader(self, server_ids):
        self.jumpToAddress(server_ids, [COMM_FIRMWARE_OFFSET for sid in server_ids])
        time.sleep(0.5)
        self._ser.read_all()

    def enterBootloader(self, server_ids):
        self.resetSystem(server_ids)

    def readRegisters(self, server_ids, start_addr, count):
        responses = self.doTransaction(server_ids, [COMM_FC_REG_READ]*len(server_ids), [struct.pack('<HB', addr, ct) for addr, ct in zip(start_addr, count)])
        data = [response[1] for response in responses]
        return data

    def writeRegisters(self, server_ids, start_addr, count, data):
        responses = self.doTransaction(server_ids, [COMM_FC_REG_WRITE]*len(server_ids), [struct.pack('<HB', addr, ct) + dat for addr, ct, dat in zip(start_addr, count, data)])
        success = [response[0] for response in responses]
        return success 
    
    def readWriteRegisters(self, server_ids, read_start_addr, read_count, write_start_addr, write_count, write_data):
        message = [struct.pack('<HBHB', readsa, readct, writesa, writect) + writed for readsa, readct, writesa, writect, writed in zip(read_start_addr, read_count, write_start_addr, write_count, write_data)]
        responses = self.doTransaction(server_ids, [COMM_FC_REG_READ_WRITE]*len(server_ids), message)
        data = [response[1] for response in responses]
        return data

    def resetSystem(self, server_ids):
        self.writeRequest(server_ids, [COMM_FC_SYSTEM_RESET]*len(server_ids))
        return True

    def jumpToAddress(self, server_ids, jump_addr=[COMM_FIRMWARE_OFFSET]):
        self.writeRequest(server_ids, [COMM_FC_JUMP_TO_ADDR]*len(server_ids), [struct.pack('<I', addr) for addr in jump_addr])
        return True

    def getFlashSectorCount(self, server_ids):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_SECTOR_COUNT]*len(server_ids), ['' for sid in server_ids])
        data = [resp[1] for resp in responses]
        return [struct.unpack('<I', dat)[0] for dat in data]

    def getFlashSectorStart(self, server_ids, sector_nums):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_SECTOR_START]*len(server_ids), [struct.pack('<I', sector) for sector in sector_nums])
        data = [resp[1] for resp in responses]
        return [struct.unpack('<I', dat)[0] for dat in data]

    def getFlashSectorSize(self, server_ids, sector_nums):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_SECTOR_SIZE]*len(server_ids), [struct.pack('<I', sector) for sector in sector_nums])
        data = [resp[1] for resp in responses]
        return [struct.unpack('<I', dat)[0] for dat in data]

    def eraseFlashSector(self, server_ids, sector_nums):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_SECTOR_ERASE]*len(server_ids), [struct.pack('<I', sector) for sector in sector_nums])
        success = [resp[0] for resp in responses]
        return success

    def programFlash(self, server_ids, dest_addr, data):
        for i in range(0, len(data), COMM_SINGLE_PROGRAM_LENGTH):
            success = self._programFlashLimitedLength(server_ids, [addr + i for addr in dest_addr], data[i:i+COMM_SINGLE_PROGRAM_LENGTH])
            if not all(success):
                return False
        return True

    def _programFlashLimitedLength(self, server_ids, dest_addr, data):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_PROGRAM]*len(server_ids), [struct.pack('<I', addr) + data for addr in dest_addr])
        success = [resp[0] for resp in responses]
        return success

    def readFlash(self, server_ids, src_addr, length):
        data = '' 
        for i in range(0, length, COMM_SINGLE_READ_LENGTH):
            read_len = min(length - i, COMM_SINGLE_READ_LENGTH)
            data_chunk = self._readFlashLimitedLength(server_ids, src_addr + i, read_len)
            if any([len(chunk) != read_len for chunk in data_chunk]):
                return False
            data += data_chunk[0]
        return data

    def _readFlashLimitedLength(self, server_ids, src_addr, length):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_READ]*len(server_ids), [struct.pack('<II', src_addr, length)])
        data = [resp[1] for resp in responses]
        print data
        return data

    def readCalibration(self, server_ids):
        l = struct.unpack('<H', self.readFlash(server_ids, COMM_NVPARAMS_OFFSET+1, 2))[0]
        b = self.readFlash(server_ids, COMM_NVPARAMS_OFFSET+3, l)
        print b
        return json.loads(b)

    def verifyFlash(self, server_ids, dest_addr, data):
        for i in range(0, len(data), COMM_SINGLE_VERIFY_LENGTH):
            success = self._verifyFlashLimitedLength(server_ids, [addr + i for addr in dest_addr], data[i:i+COMM_SINGLE_VERIFY_LENGTH])
            if not all(success):
                return False
        return True

    def _verifyFlashLimitedLength(self, server_ids, dest_addr, data):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_VERIFY]*len(server_ids), [struct.pack('<I', addr) + data for addr in dest_addr])
        success = [resp[0] for resp in responses]
        return success

    def verifyFlashErased(self, server_ids, dest_addr, length):
        responses = self.doTransaction(server_ids, [COMM_FC_FLASH_VERIFY_ERASED]*len(server_ids), [struct.pack('<II', addr, length) for addr in dest_addr])
        success = [resp[0] for resp in responses]
        return success

    def eraseFlash(self, server_ids, addr, length, sector_map=None):
        if sector_map is None:
            sector_map = self.getFlashSectorMap(server_ids)

        # Find out which sectors need to be erased
        board_sector_nums = [smap.getFlashSectorsOfAddressRange(ad, length) for smap, ad in zip(sector_map, addr)] 

        for nums in board_sector_nums[0]:
            success = self.eraseFlashSector(server_ids, [nums] * len(server_ids))
            if not success:
                return False

        return True

    def writeFlash(self, server_ids, dest_addr, data, sector_map=None, print_progress=False):
        if sector_map is None:
            sector_map = self.getFlashSectorMap(server_ids)

        if print_progress:
            print "Erasing flash"

        success = self.eraseFlash(server_ids, dest_addr, len(data), sector_map)
        if not success:
            return False

        if print_progress:
            print "Verifying flash was erased"

        success = self.verifyFlashErased(server_ids, dest_addr, len(data))
        if not success:
            return False

        if print_progress:
            print "Programming flash"

        success = self.programFlash(server_ids, dest_addr, data)
        if not success:
            return False

        if print_progress:
            print "Verifying flash was programmed"

        success = self.verifyFlash(server_ids, dest_addr, data)
        if not success:
            return False

        return True

    def getFlashSectorMap(self, server_ids):
        sector_counts = self.getFlashSectorCount(server_ids)
        sector_starts = []
        sector_sizes = []

        # TODO: Make this individual sector stars and sizes. THis does the 0th item only.
        for count in range(sector_counts[0]):
            sector_starts.append(self.getFlashSectorStart(server_ids, [count]*len(server_ids)))
            sector_sizes.append(self.getFlashSectorSize(server_ids, [count]*len(server_ids)))
        
        sector_maps = [0] * len(server_ids)
        for i in range(len(server_ids)):
            sector_maps[i] = FlashSectorMap(sector_counts[i], [sector_st[i] for sector_st in sector_starts], [sector_sz[i] for sector_sz in sector_sizes])

        return sector_maps 

    def doTransaction(self, server_ids, func_code, data):
        # Send the request to the boards.
        self.writeRequest(server_ids, func_code, data) 

        # Listen to the responses.
        responses = ['']*len(server_ids)
        for i in range(len(responses)):
            responses[i] = self.readResponse(server_ids[i], func_code[i])

        return responses

    def writeRequest(self, server_ids, func_code, data=['']):
        message = ''
        flags = COMM_FLAG_SEND
        for i in range(len(server_ids)):
            sub_message = struct.pack('<BB', server_ids[i], func_code[i]) + data[i]
            message = message + struct.pack('H', len(sub_message)) + sub_message

        prefixed_message = struct.pack('<BBBH', 0xFF, COMM_VERSION, flags, len(message)) + message
        crc = self._computeCRC(message)
        datagram = prefixed_message + struct.pack('<H', crc) 

        #print (":".join("{:02x}".format(ord(c)) for c in datagram))

        self._ser.write(datagram)

    def readResponse(self, server_id, func_code):
        sync = self._ser.read()
        if len(sync) != 1 or sync != "\xff":
            # Reached maximum number of tries
            # self._ser.flushInput()
            return False, None

        version = self._ser.read()
        if len(version) != 1 or version != "\xfe":
            # self._ser.flushInput()
            return False, None

        flags = self._ser.read()

        length = self._ser.read(2)
        if length == None or len(length) == 0:
            return False, None

        message_len, = struct.unpack('H', length)
        message = self._ser.read(message_len)
        
        #print (":".join("{:02x}".format(ord(c)) for c in message))

        if len(message) < message_len:
            # self._ser.flushInput()
            return False, None

        crc_bytes = self._ser.read(2)
        #print (":".join("{:02x}".format(ord(c)) for c in crc_bytes))

        if len(crc_bytes) < 2:
            # self._ser.flushInput()
            return False, None

        message_server_id, message_func_code, errors = struct.unpack('<BBH', message[2:6])

        if message_server_id != server_id:
            raise ProtocolError('received unexpected server ID: saw ' \
                                + str(message_server_id) + ', expected ' + str(server_id))

        if message_func_code != func_code:
            raise ProtocolError('received unexpected func ID: saw ' \
                                + str(message_func_code) + ', expected ' + str(func_code))

        message_crc, = struct.unpack('<H', crc_bytes)
        computed_crc = self._computeCRC(message)

        if message_crc != computed_crc:
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

        return success, message[6:]

    def _computeCRC(self, values):
        crc = crcmod.predefined.Crc('crc-16')
        crc.update(values)
        return crc.crcValue
