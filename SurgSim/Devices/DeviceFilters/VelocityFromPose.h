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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_VELOCITYFROMPOSE_H
#define SURGSIM_DEVICES_DEVICEFILTERS_VELOCITYFROMPOSE_H

#include <memory>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Math/KalmanFilter.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Framework/Clock.h"

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
class VelocityFromPose : public SurgSim::Input::CommonDevice,
	public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit VelocityFromPose(const std::string& name);

	bool initialize() override;

	bool finalize() override;

	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	/// The time point of the last update, used to calculate the update period.
	SurgSim::Framework::Clock::time_point m_lastTime;

	/// A copier into the input DataGroup, if needed.
	std::shared_ptr<SurgSim::DataStructures::DataGroupCopier> m_copier;

	/// The last pose received from the input data.
	SurgSim::Math::RigidTransform3d m_lastPose;

	/// The translational Kalman filter.
	SurgSim::Math::KalmanFilter m_linearFilter;

	/// The angular Kalman filter.
	SurgSim::Math::KalmanFilter m_angularFilter;
};


};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_VELOCITYFROMPOSE_H
