
#include <pluginlib/class_list_macros.h>
#include <talos_control/aruco_cartesian_controller.h>
#include <talos_control/aruco_trajectory_cartesian_controller.h>
#include <talos_control/cartesian_controller_gazebo_ref.h>
#include <talos_control/cartesian_controller_basic.h>
#include <talos_control/cartesian_controller_torso_efforts.h>
#include <talos_control/torso_effort_controller.h>

namespace position_controller {

typedef controller_ns::aruco_cartesian_controller_class ArucoController;

typedef controller_ns::gazebo_cartesian_controller_class GazeboTrajectoryController;

typedef controller_ns::aruco_trajectory_cartesian_controller_class ArucoTrajectoryController;

typedef controller_ns::cartesian_controller_class CartesianTrajectoryController;

typedef controller_ns::torso_effort_controller_class TorsoEffortController;

typedef controller_ns::cartesian_controller_torso_efforts_class CartesianControllerTorsoEfforts;

};

PLUGINLIB_EXPORT_CLASS(position_controller::ArucoController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(position_controller::GazeboTrajectoryController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(position_controller::ArucoTrajectoryController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(position_controller::CartesianTrajectoryController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(position_controller::TorsoEffortController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(position_controller::CartesianControllerTorsoEfforts, controller_interface::ControllerBase);

