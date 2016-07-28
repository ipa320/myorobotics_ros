#ifndef __MYO_MUSCLE_JOINT_INTERFACE_H
#define __MYO_MUSCLE_JOINT_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <angles/angles.h>

namespace myo_interface
{

class MyoMuscleJointHandle : public hardware_interface::JointHandle
{
	public:
		MyoMuscleJointHandle() {}
		MyoMuscleJointHandle(const hardware_interface::JointHandle& js, double* displacement, double analogIn[], bool* digitalOut)
			: hardware_interface::JointHandle(js), displacement_(displacement), analogIn_(analogIn), digitalOut_(digitalOut) {}

		double getDisplacement() const {
			return *displacement_;
		}

		double getAnalogIn(int index){
			return analogIn_[index];
		}

		void setDigitalOut(bool state){
			*digitalOut_ = state;
		}

	private:
		double* displacement_;
		double* analogIn_;
		bool* digitalOut_;
};


class MyoMuscleJointInterface : public hardware_interface::HardwareInterface
{
	public:
	/// Get the vector of joint names registered to this interface.
	std::vector<std::string> getJointNames() const
	{
		std::vector<std::string> out;
		out.reserve(handle_map_.size());
		for( HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
		{
			out.push_back(it->first);
		}
		return out;
	}

	void registerJoint(const hardware_interface::JointHandle& js, double* displacement, double analogIn[], bool* digitalOut)
	{
		MyoMuscleJointHandle handle(js, displacement, analogIn, digitalOut);
		registerHandle(handle);
	}

	void registerHandle(MyoMuscleJointHandle handle)
	{
		HandleMap::iterator it = handle_map_.find(handle.getName());
		if (it == handle_map_.end())
		{
			handle_map_.insert(std::make_pair(handle.getName(), handle));
		}
		else
		{
			it->second = handle;
		}
	}

	MyoMuscleJointHandle getHandle(const std::string& name)
	{
		HandleMap::const_iterator it = handle_map_.find(name);

		if (it == handle_map_.end())
		{
			throw hardware_interface::HardwareInterfaceException("Could not find joint [" + name + "] in MyoMuscleJointInterface");
		}

		HardwareInterface::claim(name);
		return it->second;
	}

	protected:
		typedef std::map<std::string, MyoMuscleJointHandle> HandleMap;
		HandleMap handle_map_;
};

}

#endif // ifndef __MYO_MUSCLE_JOINT_INTERFACE_H