#ifndef __EXAMPLECONTROLLER_H
#define __EXAMPLECONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>

namespace myo_controllers{

class ExampleController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:

  bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // get sensor values
    double position = joint_.getPosition();
    double velocity = joint_.getVelocity();
    double effort = joint_.getEffort();
    double displacement = joint_.getDisplacement();
    double analogIN0 = joint_.getAnalogIn(0);
    // set pwm cycle [-4000;4000]
    joint_.setCommand(200);
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  myo_interface::MyoMuscleJointHandle joint_;
};
}
#endif