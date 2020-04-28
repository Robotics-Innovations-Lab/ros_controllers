#include <traj_or_jog_controller/traj_or_jog_controller.h>

namespace traj_or_jog_controller
{

/** \brief Override the initializer of the base class. */
template <class SegmentImpl, class HardwareInterface>
bool TrajOrJogController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
          ros::NodeHandle&   root_nh,
          ros::NodeHandle&   controller_nh)
{
  // Initialize the base class, JointTrajectoryController
  JointTrajectoryController::init(hw, root_nh, controller_nh);

  // Add a subscriber for real-time velocity commands
  velocity_command_sub_ = JointTrajectoryController::
    controller_nh_.subscribe("velocity_command", 1, &TrajOrJogController::velocityCommandCB, this);

  n_joints_ = JointTrajectoryController::joint_names_.size();

  commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

  allow_trajectory_execution_ = false;
  trajectory_is_active_ = false;

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void TrajOrJogController<SegmentImpl, HardwareInterface>::update(const ros::Time& time, const ros::Duration& period)
{
  // When updating, the real-time velocity controller takes priority over the trajectory controller.
  // The real-time velocity controller takes over as soon as a trajectory completes, and a new real-time velocity
  // command can interrupt a trajectory at any time.
  // The member variable allow_trajectory_execution_ determines which controller gets updated.

  // If trajectory execution is not active
  if ( !allow_trajectory_execution_ )
  {
    JointTrajectoryController::preemptActiveGoal();
    std::vector<double> & command = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; ++i)
    {
      JointTrajectoryController::desired_state_.velocity[i] = command[i];
      JointTrajectoryController::desired_state_.position[i] = JointTrajectoryController::current_state_.position[i] + period.toSec() * command[i];

      JointTrajectoryController::state_error_.position[i] =
        angles::shortest_angular_distance(JointTrajectoryController::current_state_.position[i],
                                          JointTrajectoryController::desired_state_.position[i]);
      JointTrajectoryController::state_error_.velocity[i] = JointTrajectoryController::desired_state_.velocity[i] -
                                                          JointTrajectoryController::joints_[i].getVelocity();
      JointTrajectoryController::state_error_.acceleration[i] = 0.0;

      JointTrajectoryController::hw_iface_adapter_.updateCommand(JointTrajectoryController::time_data_.readFromRT()->uptime + period, period,
                                                             JointTrajectoryController::desired_state_,
                                                             JointTrajectoryController::state_error_);
    }
  }
  // If trajectory execution is allowed
  else
  {
    // Update the base class, JointTrajectoryController
    JointTrajectoryController::update(time, period);

    // Back to real-time velocity control if we see the trajectory finish
    // The rt_active_goal_ can lag allow_trajectory_execution_, so we want to make sure we watch it True -> False
    if (trajectory_is_active_ && JointTrajectoryController::rt_active_goal_ == NULL)
    {
      TrajOrJogController::allow_trajectory_execution_ = false;
    }
  }

  // Update the tracking of rt_active_goal_ pointer
  trajectory_is_active_ = JointTrajectoryController::rt_active_goal_ != NULL;
}

/**
 * \brief Override the callback for the JointTrajectoryController action server.
 */
template <class SegmentImpl, class HardwareInterface>
void TrajOrJogController<SegmentImpl, HardwareInterface>::
goalCB(GoalHandle gh)
{
  // Make sure trajectory execution is enabled.
  // It will be interrupted by any new real-time commands.
  if (!allow_trajectory_execution_)
  {
    // Reset the JointTrajectoryController to ensure it has current joint angles, etc.
    JointTrajectoryController::starting(ros::Time::now());

    allow_trajectory_execution_ = true;
  }

  JointTrajectoryController::goalCB(gh);
}

}  // namespace traj_or_jog_controller

// Set up namespacing of controllers and create their plugins.
namespace velocity_controllers
{
  /**
   * \brief A combination of a JointTrajectoryController with a ForwardJointGroupCommand controller.
   */
  typedef traj_or_jog_controller::TrajOrJogController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::VelocityJointInterface>
          TrajOrJogController;
}

namespace position_controllers
{
  /**
   * \brief A combination of a JointTrajectoryController with a ForwardJointGroupCommand controller.
   */
  typedef traj_or_jog_controller::TrajOrJogController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PositionJointInterface>
          TrajOrJogController;
}

PLUGINLIB_EXPORT_CLASS(velocity_controllers::TrajOrJogController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(position_controllers::TrajOrJogController, controller_interface::ControllerBase)