#include "motor_state.h"

/*--------------------------------------------------------------------
 * MotorState()
 * Constructor.
 *------------------------------------------------------------------*/

MotorState::MotorState()
{
} // end MotorState()

/*--------------------------------------------------------------------
 * ~MotorState()
 * Destructor.
 *------------------------------------------------------------------*/

MotorState::~MotorState()
{
} // end ~MotorState()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void MotorState::publishMessage(ros::Publisher *pub_message)
{
  motor_state::motor_state_data msg;
  msg.message = message;
  msg.a = a;
  msg.b = b;

  pub_message->publish(msg);
} // end publishMessage()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void MotorState::messageCallback(const node_example::node_example_data::ConstPtr &msg)
{
  message = msg->message;
  a = msg->a;
  b = msg->b;

  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  ROS_INFO("message is %s", message.c_str());
  ROS_INFO("sum of a + b = %d", a + b);
} // end publishCallback()

/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void MotorState::configCallback(node_example::node_example_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message = config.message.c_str();
  a = config.a;
  b = config.b;
} // end configCallback()
