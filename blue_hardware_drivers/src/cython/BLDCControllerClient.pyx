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

        #  #  queuePacket(int board_id, Packet* packet);
        #
        #  # Initialize Board ID for Disco Bus Protocol
        #  queueConfirmID(int board_id);
        #  queueEnumerate(int board_id);
        #  getEnumerateResponse(int board_id, int* response_id);
        #
        #  # Program Counter Adjustments
        #  #  queueLeaveBootloader(int board_id, uint32_t jump_addr);
        #
        #  # Calibration Setup
        #  #  queueSetTimeout(int board_id, uint16_t value);
        #  #  queueSetControlMode(int board_id, comm_ctrl_mode_t control_mode);
        #  #  queueSetZeroAngle(int board_id, uint16_t value);
        #  #  queueSetERevsPerMRev(int board_id, uint8_t value);
        #  #  queueSetInvertPhases(int board_id, uint8_t value);
        #  queueSetTorqueConstant(int board_id, float value);
        #  queueSetPositionOffset(int board_id, float value);
        #  queueSetEACScale(int board_id, float value);
        #  queueSetEACOffset(int board_id, float value);
        #  #  queueSetEACTable(int board_id, size_t start_index, uint8_t *values, size_t count);
        #  queueSetDirectCurrentControllerKp(int board_id, float value);
        #  queueSetDirectCurrentControllerKi(int board_id, float value);
        #  queueSetQuadratureCurrentControllerKp(int board_id, float value);
        #  queueSetQuadratureCurrentControllerKi(int board_id, float value);
        #  queueSetVelocityControllerKp(int board_id, float value);
        #  queueSetVelocityControllerKi(int board_id, float value);
        #  queueSetPositionControllerKp(int board_id, float value);
        #  queueSetPositionControllerKi(int board_id, float value);
        #  queueSetIAOffset(int board_id, float value);
        #  queueSetIBOffset(int board_id, float value);
        #  queueSetICOffset(int board_id, float value);
        #
        #  # Drive Commands
        #  queueSetCommand(int board_id, float value);
        #  queueSetPosCommand(int board_id, float position, float feedforward);
        #  queueGetRotorPosition(int board_id);
        #  queueSetCommandAndGetRotorPosition(int board_id, float value);
        #  queueSetPositionAndGetRotorPosition(int board_id, float value);
        #  queueGetState(int board_id);
        #  queueSetCommandAndGetState(int board_id, float value);
        #  queueSetPosCommandAndGetState(int board_id, float position, float feedforward);

        # Init Motor State Commands
        #  queueSetRevolutions(int board_id, int16_t value);

        # Send queued packets and receive from boards
        exchange();

        # Remove all items currently in the queue
        clearQueue();

        # Send all boards to bootloader
        resetBoards();

        # Clear the RS485 Buffer
        resetBuffer();

        #  # Check if a watchdog reset has occurred on a given board
        #  bool checkWDGRST(int board_id);
        #  bool checkTimeout(int board_id);
        #  queueClearWDGRST(int board_id);
        #
        #  # Result Commands
        #  resultGetRotorPosition(int board_id, float* result);
        #  #  resultGetState(int board_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, int32_t* acc_x, int32_t* acc_y, int32_t* acc_z);
        #
        #  # Setup/Programming Commands
        #  bool initMotor(int board_id);

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

    #  def queue_get_rotor_position(self, board_id):
        #  self._bldc_client.queueGetRotorPosition(board_id)

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

    #  def result_get_rotor_position(self, board_id, result):
    #      self._bldc_client.resultGetRotorPosition(board_id, result)
