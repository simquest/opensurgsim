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

#ifndef SURGSIM_DEVICES_MULTIAXIS_POSEINTEGRATOR_H
#define SURGSIM_DEVICES_MULTIAXIS_POSEINTEGRATOR_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{
namespace Device
{
/// A device filter that integrates the pose, turning a relative device into an absolute one.
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
	/// \param inputData An initial value for the application's input from the device (i.e., pose).
	PoseIntegrator(const std::string& name, const SurgSim::DataStructures::DataGroup& inputData);

	/// Integrates the pose.
	/// \param pose	The latest differential pose.
	/// \return	The integrated pose.
	const PoseType& integrate(const PoseType& pose);

	virtual bool initialize() override;

	virtual bool finalize() override;

	bool isInitialized() const;

	virtual void initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& newInputData) override;

	virtual void handleInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& newInputData) override;

	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	PoseType m_poseResult;
};


};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_POSEINTEGRATOR_H
