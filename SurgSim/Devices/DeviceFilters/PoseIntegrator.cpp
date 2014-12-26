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
namespace Device
{

PoseIntegrator::PoseIntegrator(const std::string& name) :
	CommonDevice(name),
	m_poseResult(PoseType::Identity()),
	m_firstInput(true),
	m_poseIndex(-1),
	m_linearVelocityIndex(-1),
	m_angularVelocityIndex(-1),
	m_resetIndex(-1)
{
}

const PoseIntegrator::PoseType& PoseIntegrator::integrate(const PoseType& pose)
{
	// Note: we apply translation and rotation separately.  This is NOT the same as (m_poseResult * pose)!
	m_poseResult.pretranslate(pose.translation());
	m_poseResult.rotate(pose.rotation());
	return m_poseResult;
}

bool PoseIntegrator::initialize()
{
	return true;
}

bool PoseIntegrator::finalize()
{
	return true;
}

void PoseIntegrator::initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	if (m_firstInput)
	{
		m_firstInput = false;

		if (!inputData.vectors().hasEntry(SurgSim::DataStructures::Names::LINEAR_VELOCITY) ||
			!inputData.vectors().hasEntry(SurgSim::DataStructures::Names::ANGULAR_VELOCITY))
		{
			SurgSim::DataStructures::DataGroupBuilder builder;
			builder.addEntriesFrom(inputData);
			builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
			builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
			getInputData() = builder.createData();
			m_copier = std::make_shared<SurgSim::DataStructures::DataGroupCopier>(inputData, &getInputData());
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

	m_poseIndex = inputData.poses().getIndex(SurgSim::DataStructures::Names::POSE);
	m_linearVelocityIndex = getInputData().vectors().getIndex(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
	m_angularVelocityIndex = getInputData().vectors().getIndex(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
	m_resetIndex = inputData.booleans().getIndex(m_resetName);

	SurgSim::Math::RigidTransform3d pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		m_poseResult = pose;
	}
}

void PoseIntegrator::handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	if (m_copier == nullptr)
	{
		getInputData() = inputData;
	}
	else
	{
		m_copier->copy(inputData, &getInputData());
	}

	if (m_poseIndex >= 0)
	{
		SurgSim::Math::RigidTransform3d pose;
		if (inputData.poses().get(m_poseIndex, &pose))
		{
			m_timer.markFrame();
			double rate = m_timer.getAverageFrameRate();
			if (m_timer.getNumberOfClockFails() > 0)
			{
				m_timer.start();
				rate = 0.0;
				SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("Devices/Filters/PoseIntegrator")) <<
					"The Timer used by " << getName() <<
					" had a clock fail.  The calculated velocities will be zero this update.";
			}

			if (!boost::math::isnormal(rate))
			{
				rate = 0.0;
			}

			if (m_resetIndex >= 0)
			{
				bool reset = false;
				inputData.booleans().get(m_resetName, &reset);
				if (reset)
				{
					pose.translation() = -m_poseResult.translation();
					pose.linear() = m_poseResult.linear().transpose();
				}
			}

			// Before updating the current pose, use it to calculate the angular velocity.
			Vector3d rotationAxis;
			double angle;
			SurgSim::Math::computeAngleAndAxis(pose.rotation(), &angle, &rotationAxis);
			rotationAxis = m_poseResult.rotation() * rotationAxis; // rotate the axis into global space
			// The angular and linear indices must exist because the entries were added in initializeInput.
			getInputData().vectors().set(m_angularVelocityIndex, rotationAxis * angle * rate);

			getInputData().poses().set(m_poseIndex, integrate(pose));

			getInputData().vectors().set(m_linearVelocityIndex, pose.translation() * rate);
		}
	}
	pushInput();
}

bool PoseIntegrator::requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData)
{
	bool state = pullOutput();
	if (state)
	{
		*outputData = getOutputData();
	}
	return state;
}

void PoseIntegrator::setReset(const std::string& name)
{
	SURGSIM_ASSERT(m_firstInput) <<
		"PoseIntegrator::setReset cannot be called after the first call to initializeInput.";
	m_resetName = name;
}

};  // namespace Device
};  // namespace SurgSim
