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

#include "SurgSim/Devices/DeviceFilters/PoseIntegrator.h"

#include <boost/math/special_functions/fpclassify.hpp>

#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/DataStructures/DataGroupCopier.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::PoseIntegrator, PoseIntegrator);

PoseIntegrator::PoseIntegrator(const std::string& name) :
	DeviceFilter(name),
	m_poseResult(PoseType::Identity())
{
}

const PoseIntegrator::PoseType& PoseIntegrator::integrate(const PoseType& pose)
{
	// Note: we apply translation and rotation separately.  This is NOT the same as (m_poseResult * pose)!
	m_poseResult.pretranslate(pose.translation());
	m_poseResult.rotate(pose.rotation());
	return m_poseResult;
}

void PoseIntegrator::initializeInput(const std::string& device, const DataStructures::DataGroup& inputData)
{
	if (getInputData().isEmpty())
	{
		if (!inputData.vectors().hasEntry(DataStructures::Names::LINEAR_VELOCITY) ||
			!inputData.vectors().hasEntry(DataStructures::Names::ANGULAR_VELOCITY))
		{
			DataStructures::DataGroupBuilder builder;
			builder.addEntriesFrom(inputData);
			builder.addVector(DataStructures::Names::LINEAR_VELOCITY);
			builder.addVector(DataStructures::Names::ANGULAR_VELOCITY);
			getInputData() = builder.createData();
			m_copier = std::make_shared<DataStructures::DataGroupCopier>(inputData, &getInputData());
		}
	}

	if (m_copier == nullptr)
	{
		getInputData() = inputData;
	}
	else
	{
		m_copier->copy(inputData, &getInputData());
	}

	PoseType pose;
	if (inputData.poses().get(DataStructures::Names::POSE, &pose))
	{
		m_poseResult = pose;
	}
}

void PoseIntegrator::handleInput(const std::string& device, const DataStructures::DataGroup& inputData)
{
	if (m_copier == nullptr)
	{
		getInputData() = inputData;
	}
	else
	{
		m_copier->copy(inputData, &getInputData());
	}

	PoseType pose;
	if (inputData.poses().get(DataStructures::Names::POSE, &pose))
	{
		m_timer.markFrame();
		double rate = m_timer.getAverageFrameRate();
		if (m_timer.getNumberOfClockFails() > 0)
		{
			m_timer.start();
			rate = 0.0;
			SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Devices/Filters/PoseIntegrator")) <<
				"The Timer used by " << getName() <<
				" had a clock fail.  The calculated velocities will be zero this update.";
		}

		if (!boost::math::isnormal(rate))
		{
			rate = 0.0;
		}

		bool reset = false;
		inputData.booleans().get(m_resetName, &reset);
		if (reset)
		{
			pose.translation() = -m_poseResult.translation();
			pose.linear() = m_poseResult.linear().transpose();
		}

		SurgSim::Math::RigidTransform3d resetPose;
		if (m_resetPose.hasValue())
		{
			m_poseResult.translation() = m_resetPose.getValue().translation();
			m_poseResult.linear() = m_resetPose.getValue().linear();
			m_resetPose.invalidate();
		}

		// Before updating the current pose, use it to calculate the angular velocity.
		Vector3d rotationAxis;
		double angle;
		Math::computeAngleAndAxis(pose.rotation(), &angle, &rotationAxis);
		rotationAxis = m_poseResult.rotation() * rotationAxis; // rotate the axis into global space
		getInputData().vectors().set(DataStructures::Names::ANGULAR_VELOCITY, rotationAxis * angle * rate);
		getInputData().poses().set(DataStructures::Names::POSE, integrate(pose));
		getInputData().vectors().set(DataStructures::Names::LINEAR_VELOCITY, pose.translation() * rate);
	}
	pushInput();
}

void PoseIntegrator::setReset(const std::string& name)
{
	SURGSIM_ASSERT(getInputData().isEmpty()) <<
		"PoseIntegrator::setReset cannot be called after the first call to initializeInput.";
	m_resetName = name;
}

void PoseIntegrator::filterOutput(const std::string& device, 
	const SurgSim::DataStructures::DataGroup& dataToFilter, 
	SurgSim::DataStructures::DataGroup* result)
{
	SurgSim::Math::RigidTransform3d newPose;
	if (dataToFilter.poses().hasData(SurgSim::DataStructures::Names::POSE))
	{
		dataToFilter.poses().get(SurgSim::DataStructures::Names::POSE, &newPose);
		m_resetPose = newPose;
	}
	
	*result = dataToFilter;
}


};  // namespace Devices
};  // namespace SurgSim
