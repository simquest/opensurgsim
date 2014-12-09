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
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
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
class VelocityFromPose : public SurgSim::Input::CommonDevice,
	public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	/// The type used for poses.
	typedef SurgSim::Math::RigidTransform3d PoseType;

	/// Constructor.
	/// \param name	Name of this device filter.
	explicit VelocityFromPose(const std::string& name);

	virtual bool initialize() override;

	virtual bool finalize() override;

	virtual void initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& inputData) override;

	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	/// Calculate velocity via Kalman filter.
	/// \param pose	The latest differential pose.
	/// \param period The time duration since the previous update.
	/// \param linearVelocity The linear velocity.
	/// \param angularVelocity The angular velocity.
	void calculateVelocity(const PoseType& pose, double period, SurgSim::Math::Vector3d* linearVelocity,
		SurgSim::Math::Vector3d* angularVelocity);

	void kalman(const SurgSim::Math::Vector3d& measurement, double period, Eigen::Matrix<double, 9, 1>* state,
		Eigen::Matrix<double, 9, 9, Eigen::RowMajor>* covariance);
	
	/// A timer for the update rate needed for calculating velocity.
	SurgSim::Framework::Timer m_timer;

	/// true if the input DataGroup should be created.
	bool m_firstInput;

	/// A copier into the input DataGroup, if needed.
	std::shared_ptr<SurgSim::DataStructures::DataGroupCopier> m_copier;


	Eigen::Matrix<double, 9, 1> m_linearState;
	Eigen::Matrix<double, 9, 9, Eigen::RowMajor> m_linearCovariance;
};


};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_VELOCITYFROMPOSE_H
