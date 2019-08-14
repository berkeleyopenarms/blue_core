#include "blue_hardware_drivers/comms_defs.h"
#include "json/json.h"

/*
bool initMotor(comm_id_t board_id){
  uint32_t len = 0;
  std::string data = "";

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Calibrating board: " << (int) board_id << std::endl;
#endif

  readFlash(board_id, COMM_NVPARAMS_OFFSET, 2, data);
#ifdef DEBUG_CALIBRATION_DATA
  for (unsigned char c : data)
    printf("%02x:", c);
  std::cout << std::endl;
#endif

  std::stringstream buf;
  buf << data;
  buf.read(reinterpret_cast<char*>(&len), sizeof(len));

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Calibration length: " << len << std::endl;
#endif
  data = "";
  readFlash(board_id, COMM_NVPARAMS_OFFSET+2, len, data);
  
#ifdef DEBUG_CALIBRATION_DATA
  std::cout << data << std::endl;
#endif

  Json::Reader reader;
  Json::Value calibrations;
  reader.parse(data, calibrations);

  if ( calibrations["angle"].isNull()
    || calibrations["inv"].isNull()
    || calibrations["epm"].isNull()
    || calibrations["torque"].isNull()
    || calibrations["zero"].isNull()
    || calibrations["ia_off"].isNull()
    || calibrations["ib_off"].isNull()
    || calibrations["ic_off"].isNull()
    ) {
    return false;
  }

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Zero Angle: " << calibrations["angle"].asUInt() << std::endl;
#endif
    queueSetZeroAngle(board_id, (uint16_t) calibrations["angle"].asUInt());
    exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Invert Phases: " << calibrations["inv"].asUInt() << std::endl;
#endif
  queueSetInvertPhases(board_id, (uint8_t) calibrations["inv"].asUInt());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Encoder Revs Per Magnetic Revolution: " << calibrations["epm"].asUInt() << std::endl;
#endif
  queueSetERevsPerMRev(board_id, (uint8_t) calibrations["epm"].asUInt());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Torque Constant: " << calibrations["torque"].asFloat() << std::endl;
#endif
  queueSetTorqueConstant(board_id, calibrations["torque"].asFloat());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Position Offset: " << calibrations["zero"].asFloat() << std::endl;
#endif
  queueSetPositionOffset(board_id, calibrations["zero"].asFloat());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Phases A Current: " << calibrations["ia_off"].asFloat() << std::endl;
#endif
  queueSetIAOffset(board_id, calibrations["ia_off"].asFloat());
  exchange();
#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Phases B Current: " << calibrations["ib_off"].asFloat() << std::endl;
#endif
  queueSetIBOffset(board_id, calibrations["ib_off"].asFloat());
  exchange();
#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Phases C Current: " << calibrations["ic_off"].asFloat() << std::endl;
#endif
  queueSetICOffset(board_id, calibrations["ic_off"].asFloat());
  exchange();

  if (calibrations.isMember("eac_type")) {
    std::string eac_type = calibrations["eac_type"].asString();

    if (eac_type.compare("int8") == 0) {
#ifdef DEBUG_CALIBRATION_DATA
      std::cout << "EAC calibration available" << std::endl;
#endif



      Json::Value eac_table = calibrations["eac_table"];

#ifdef DEBUG_CALIBRATION_DATA
      std::cout << eac_table << std::endl;
#endif

      if (eac_table.isArray()) {
        size_t eac_table_len = eac_table.size();

        // Copy the table values into a contiguous block of memory
        std::vector<uint8_t> eac_table_values(eac_table_len);
        for (size_t i = 0; i < eac_table_len; i++) {
          eac_table_values[i] = eac_table[(unsigned int) i].asInt();
        }

        size_t slice_len = COMM_SINGLE_SET_EAC_TABLE_LENGTH;
        for (size_t i = 0; i < eac_table_len; i += slice_len) {
          queueSetEACTable(board_id, i, &eac_table_values[i], std::min(slice_len, eac_table_len - i));
          exchange();
        }
      }

      queueSetEACOffset(board_id, calibrations["eac_offset"].asFloat());
      exchange();

      queueSetEACScale(board_id, calibrations["eac_scale"].asFloat());
      exchange();
    } else {

#ifdef DEBUG_CALIBRATION_DATA
      std::cout << "Unsupported EAC type \"" << eac_type << "\", ignoring" << std::endl;
#endif
    }
  }

  if (calibrations["torque"].asFloat() > 1) {
    queueSetDirectCurrentControllerKp(board_id, 0.5f);
    exchange();
    queueSetDirectCurrentControllerKi(board_id, 0.1f);
    exchange();
    queueSetQuadratureCurrentControllerKp(board_id, 1.0f);
    exchange();
    queueSetQuadratureCurrentControllerKi(board_id, 0.2f);
    exchange();
  } else {
    queueSetDirectCurrentControllerKp(board_id, 0.5f);
    exchange();
    queueSetDirectCurrentControllerKi(board_id, 0.1f);
    exchange();
    queueSetQuadratureCurrentControllerKp(board_id, 1.0f);
    exchange();
    queueSetQuadratureCurrentControllerKi(board_id, 0.2f);
    exchange();
  }

  return true;
}
*/
