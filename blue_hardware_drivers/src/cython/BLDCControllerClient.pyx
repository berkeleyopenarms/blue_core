# distutils: language = c++
# cython: c_string_type=unicode, c_string_encoding=utf8

from libcpp cimport bool
import cython
from libc.stdlib cimport malloc, free
from libcpp.string cimport string
from libcpp.vector cimport vector

#  cdef extern from "blue_hardware_drivers/comms_defs.h" namespace "blue_hardware_drivers":
    #  cdef comm_id_t

ctypedef unsigned char comm_id_t

cdef extern from "blue_hardware_drivers/BLDCControllerClient.hpp" namespace "blue_hardware_drivers":
    cppclass BLDCControllerClient:
        BLDCControllerClient(string port, vector[comm_id_t] boards) except +

        #  # Initialize Board ID for Disco Bus Protocol
        queueConfirmID(comm_id_t board_id);
        queueEnumerate(comm_id_t board_id);
        getEnumerateResponse(comm_id_t board_id, int* response_id);

        #  # Program Counter Adjustments
        #  queueLeaveBootloader(comm_id_t board_id, uint32_t jump_addr);

        # Calibration Setup
        #  queueSetTimeout(comm_id_t board_id, uint16_t value);
        #  queueSetControlMode(comm_id_t board_id, comm_ctrl_mode_t control_mode);
        #  queueSetZeroAngle(comm_id_t board_id, uint16_t value);
        #  queueSetERevsPerMRev(comm_id_t board_id, uint8_t value);
        #  queueSetInvertPhases(comm_id_t board_id, uint8_t value);
        #  queueSetTorqueConstant(comm_id_t board_id, float value);
        #  queueSetPositionOffset(comm_id_t board_id, float value);
        #  queueSetEACScale(comm_id_t board_id, float value);
        #  queueSetEACOffset(comm_id_t board_id, float value);
        #  queueSetEACTable(comm_id_t board_id, size_t start_index, uint8_t *values, size_t count);
        queueSetDirectCurrentControllerKp(comm_id_t board_id, float value);
        queueSetDirectCurrentControllerKi(comm_id_t board_id, float value);
        queueSetQuadratureCurrentControllerKp(comm_id_t board_id, float value);
        queueSetQuadratureCurrentControllerKi(comm_id_t board_id, float value);
        queueSetVelocityControllerKp(comm_id_t board_id, float value);
        queueSetVelocityControllerKi(comm_id_t board_id, float value);
        queueSetPositionControllerKp(comm_id_t board_id, float value);
        queueSetPositionControllerKi(comm_id_t board_id, float value);
        #  queueSetIAOffset(comm_id_t board_id, float value);
        #  queueSetIBOffset(comm_id_t board_id, float value);
        #  queueSetICOffset(comm_id_t board_id, float value);

        #  # Drive Commands
        queueSetCommand(comm_id_t board_id, float value);
        queueSetPosCommand(comm_id_t board_id, float position, float feedforward);
        queueGetRotorPosition(comm_id_t board_id);
        queueSetCommandAndGetRotorPosition(comm_id_t board_id, float value);
        queueSetPositionAndGetRotorPosition(comm_id_t board_id, float value);
        queueGetState(comm_id_t board_id);
        queueSetCommandAndGetState(comm_id_t board_id, float value);
        queueSetPosCommandAndGetState(comm_id_t board_id, float position, float feedforward);

        # Init Motor State Commands
        #  queueSetRevolutions(comm_id_t board_id, int16_t value);

        # Send queued packets and receive from boards
        exchange();

        # Remove all items currently in the queue
        clearQueue();

        # Send all boards to bootloader
        resetBoards();

        # Clear the RS485 Buffer
        resetBuffer();

        #  # Check if a watchdog reset has occurred on a given board
        bool checkWDGRST(comm_id_t board_id);
        bool checkTimeout(comm_id_t board_id);
        queueClearWDGRST(comm_id_t board_id);

        #  # Result Commands
        resultGetRotorPosition(comm_id_t board_id, float* result);
        #  resultGetState(comm_id_t board_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, int32_t* acc_x, int32_t* acc_y, int32_t* acc_z);

        #  # Setup/Programming Commands
        bool initMotor(comm_id_t board_id);

cdef class PyBLDCControllerClient:
    cdef BLDCControllerClient*c_bldc_client

    def __cinit__(
        PyBLDCControllerClient self,
        string c_port,
        boards
    ):
        cdef vector[comm_id_t] c_boards
        for i in range(len(boards)):
            c_boards.push_back(i)
        self.c_bldc_client = new BLDCControllerClient(c_port, c_boards)

    def queue_get_rotor_position(self, comm_id_t board_id):
        self._bldc_client.queueGetRotorPosition(board_id)

    def __dealloc__(self):
        del self.c_bldc_client

    def exchange(self):
        self._bldc_client.exchange()

    def clear_queue(self):
        self._bldc_client.clearQueue()

    def reset_boards(self):
        self._bldc_client.resetBoards()

    def reset_buffer(self):
        self._bldc_client.resetBuffer()

    def init_motor(self, board_id):
        self._bldc_client.initMotor(board_id)

    def result_get_rotor_position(self, board_id, result):
        self._bldc_client.resultGetRotorPosition(board_id, result)
