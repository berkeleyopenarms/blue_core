#ifndef BLUE_DYNAMICS_H
#define BLUE_DYNAMICS_H

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>

class BlueDynamics
{
public:

  BlueDynamics();

  void init(
      std::string robot_description,
      std::string baselink,
      std::string endlink);
  void setGravityVector(
      const std::vector<double> &gravity_vector);

  std::vector<double> computeGravityComp(
      const std::vector<double> &joint_pos,
      const std::vector<double> &joint_vel);
  std::vector<double> computeInverseDynamics(
      const std::vector<double> &joint_pos,
      const std::vector<double> &joint_vel,
      const std::vector<double> &target_joint_accel);

private:

  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainIdSolver_RNE> kdl_id_solver_;

};

#endif // BLUE_DYNAMICS

