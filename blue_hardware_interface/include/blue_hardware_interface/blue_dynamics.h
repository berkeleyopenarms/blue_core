#ifndef BLUE_DYNAMICS_H
#define BLUE_DYNAMICS_H

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/segment.hpp>

class BlueDynamics
{
public:

  BlueDynamics();

  void init(std::string robot_description);

private:

};

#endif // BLUE_DYNAMICS

