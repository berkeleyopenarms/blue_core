#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "node_example/node_example_data.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <node_example/node_example_paramsConfig.h>

using std::string;

class MotorState
{
public:
  //! Constructor.
  MotorState();

  //! Destructor.
  ~MotorState();

  //! Callback function for dynamic reconfigure server.
  void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);

  //! Callback function for subscriber.
  void messageCallback(const node_example::node_example_data::ConstPtr &msg);

  //! The actual message.
  string message;

  //! The first integer to use in addition.
  int a;

  //! The second integer to use in addition.
  int b;
};

#endif // SR_NODE_EXAMPLE_CORE_H
