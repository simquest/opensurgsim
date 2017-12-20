// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_POSEINTEGRATOR_H
#define SURGSIM_DEVICES_DEVICEFILTERS_POSEINTEGRATOR_H

#include <memory>
#include <string>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace DataStructures
{
class DataGroupCopier;
};

namespace Devices
{

SURGSIM_STATIC_REGISTRATION(PoseIntegrator);

/// A device filter that integrates the pose, turning a relative device into an absolute one.
/// Also provides the instantaneous linear and angular velocities.
/// \sa	SurgSim::Input::CommonDevice
/// \sa	SurgSim::Input::InputConsumerInterface
/// \sa	SurgSim::Input::OutputProducerInterface
class PoseIntegrator : public DeviceFilter
{
public:
	/// The type used for poses.
	typedef Math::UnalignedRigidTransform3d PoseType;

	/// Constructor.
	/// \param name	Name of this device filter.
	explicit PoseIntegrator(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Devices::PoseIntegrator);

	/// Integrates the pose.
	/// \param pose	The latest differential pose.
	/// \return	The integrated pose.
	const PoseType& integrate(const PoseType& pose);

	void initializeInput(const std::string& device, const DataStructures::DataGroup& inputData) override;

	/// Notifies the consumer that the application input coming from the device has been updated.
	/// Treats the pose coming from the input device as a delta pose and integrates it to get the output pose.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	void handleInput(const std::string& device, const DataStructures::DataGroup& inputData) override;

	/// Sets the string name of the boolean entry that will reset the pose to its initial value.  Such a reset can be
	/// useful if the integrated pose is used to position an object and the integration takes the object out of view.
	/// \param name The name of the NamedData<bool> entry.
	/// \warning A pose reset may generate high velocities, and if the pose is the input to a VirtualToolCoupler then
	///		large forces will likely be generated.
	/// \exception Asserts if called after initialize.
	void setReset(const std::string& name);

private:
	/// The result of integrating the input poses.
	PoseType m_poseResult;

	/// A timer for the update rate needed for calculating velocity.
	Framework::Timer m_timer;

	/// A copier into the input DataGroup, if needed.
	std::shared_ptr<DataStructures::DataGroupCopier> m_copier;

	/// The name of the reset boolean (if any).
	std::string m_resetName;
};


};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_POSEINTEGRATOR_H
