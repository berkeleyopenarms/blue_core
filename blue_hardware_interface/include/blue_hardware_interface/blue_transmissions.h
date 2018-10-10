#ifndef BLUE_TRANSMISSIONS_H
#define BLUE_TRANSMISSIONS_H

#include "blue_hardware_interface/blue_hardware_interface.h"

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include "blue_hardware_drivers/BLDCDriver.h"
#include "blue_msgs/MotorState.h"
#include "blue_msgs/JointStartupCalibration.h"

namespace ti = transmission_interface;

class BlueTransmissions
{
public:

  BlueTransmissions(ros::NodeHandle &nh);
};

#endif // BLUE_TRANSMISSIONS

