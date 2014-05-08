// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "Surgsim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace DataStructures
{
class DataGroupCopier;
};

namespace Device
{
/// A device filter that integrates the pose, turning a relative device into an absolute one.
/// Also provides the instantaneous linear and angular velocities.
/// \sa	SurgSim::Input::CommonDevice
/// \sa	SurgSim::Input::InputConsumerInterface
/// \sa	SurgSim::Input::OutputProducerInterface
class PoseIntegrator : public SurgSim::Input::CommonDevice,
	public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	/// The type used for poses.
	typedef SurgSim::Math::RigidTransform3d PoseType;

	/// Constructor.
	/// \param name	Name of this device filter.
	explicit PoseIntegrator(const std::string& name);

	/// Integrates the pose.
	/// \param pose	The latest differential pose.
	/// \return	The integrated pose.
	const PoseType& integrate(const PoseType& pose);

	virtual bool initialize() override;

	virtual bool finalize() override;

	virtual void initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& inputData) override;

	/// Notifies the consumer that the application input coming from the device has been updated.
	/// Treats the pose coming from the input device as a delta pose and integrates it to get the output pose.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	/// The result of integrating the input poses.
	PoseType m_poseResult;

	/// A timer for the update rate needed for calculating velocity.
	SurgSim::Framework::Timer m_timer;

	/// true if the input DataGroup should be created.
	bool m_firstInput;

	/// A copier into the input DataGroup, if needed.
	std::shared_ptr<SurgSim::DataStructures::DataGroupCopier> m_copier;

	///@{
	/// The indices into the DataGroups.
	int m_poseIndex;
	int m_linearVelocityIndex;
	int m_angularVelocityIndex;
	///@}
};


};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_POSEINTEGRATOR_H
