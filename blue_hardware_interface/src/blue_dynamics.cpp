#include "blue_hardware_interface/blue_dynamics.h"


BlueDynamics::BlueDynamics() {}

void BlueDynamics::init(
    std::string robot_description,
    std::string baselink,
    std::string endlink) {

  // Build KDL chain for the arm
  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  assert(kdl_parser::treeFromString(robot_description, kdl_tree));
  assert(kdl_tree.getChain(baselink, endlink, kdl_chain));

  // Clean up kdl chain
  int ns = kdl_chain.getNrOfSegments();
  for(int i = 0; i < ns; i++){
    KDL::Segment kdl_seg = kdl_chain.segments[i];

    // Only add to KDL chain if this is an actual joint
    if (kdl_seg.getJoint().getType() != KDL::Joint::None)
      kdl_chain_.addSegment(kdl_seg);
  }

}

void BlueDynamics::setGravityVector(std::vector<double> gravity_vector) {
  assert(gravity_vector.size() == 3);

  KDL::Vector kdl_gravity_vector;
  kdl_gravity_vector.data[0] = gravity_vector[0];
  kdl_gravity_vector.data[1] = gravity_vector[1];
  kdl_gravity_vector.data[2] = gravity_vector[2];

  // New gravity vector -> we need to re-create the inverse dynamics solver
  kdl_id_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_, kdl_gravity_vector));
}

void BlueDynamics::computeInverseDynamics(
    const std::vector<double> &joint_pos,
    const std::vector<double> &joint_vel,
    const std::vector<double> &target_joint_accel,
    std::vector<double> &id_torques) {

  // Convert data types for KDL
  size_t joint_count = joint_pos.size();
  KDL::JntArray kdl_joint_pos(joint_count);
  KDL::JntArray kdl_joint_vel(joint_count);
  KDL::JntArray kdl_joint_accel(joint_count);
  KDL::JntArray kdl_id_torques(joint_count);
  KDL::Wrenches kdl_wrenches;

  for (int i = 0; i < joint_count; i++) {
    kdl_joint_pos(i) = joint_pos[i];
    kdl_joint_vel(i) = joint_vel[i];
    kdl_joint_accel(i) = 0.0;
    kdl_wrenches.push_back(KDL::Wrench());
  }

  // Compute ID torques
  int statusID = kdl_id_solver_->CartToJnt(
      kdl_joint_pos,
      kdl_joint_vel,
      kdl_joint_accel,
      kdl_wrenches,
      kdl_id_torques);

  // Convert from KDL back to native C++
  for (int i = 0; i < joint_count; i++)
    id_torques[i] = kdl_id_torques.data[i];

}
