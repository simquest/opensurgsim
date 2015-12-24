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

#include "SurgSim/Devices/DeviceFilters/RecordPose.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::RecordPose, RecordPose);

RecordPose::RecordPose(const std::string& name) :
	DeviceFilter(name),
	m_cumulativeTime(0),
	m_fileName("ReplayPoseDevice.txt")
{
	m_timer.setMaxNumberOfFrames(1);
	m_timer.start();
}

RecordPose::~RecordPose()
{
	if (m_outputFile.is_open())
	{
		m_outputFile.close();
	}
}

void RecordPose::setFilename(const std::string& fileName)
{
	m_fileName = fileName;
	m_outputFile.open(m_fileName, std::ios::out | std::ios::trunc);
	SURGSIM_LOG_IF(!m_outputFile.is_open(), SurgSim::Framework::Logger::getLogger("Devices/RecordPose"), WARNING) <<
		"File " << m_fileName << " could not be open to record device pose";
}

const std::string& RecordPose::getFilename() const
{
	return m_fileName;
}

void RecordPose::filterInput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result)
{
	*result = dataToFilter;

	RigidTransform3d pose;
	if (dataToFilter.poses().get(DataStructures::Names::POSE, &pose))
	{
		m_timer.markFrame();
		m_cumulativeTime += m_timer.getLastFramePeriod();
		// We back up the time along with the pose to make sure we can replay the motion real-time, no matter
		// which rate it is recorded and replayed.
		m_outputFile << m_cumulativeTime << pose.matrix();
	}
}

};  // namespace Devices
};  // namespace SurgSim
