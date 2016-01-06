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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_RECORDPOSE_H
#define SURGSIM_DEVICES_DEVICEFILTERS_RECORDPOSE_H

#include <string>

#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_STATIC_REGISTRATION(RecordPose);

/// An input device filter that record the input pose along with the relative time. All entries in the DataGroup are
/// passed through. For convenience, it is also an OutputProducerInterface that does no filtering of the ouput data.
class RecordPose : public DeviceFilter
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit RecordPose(const std::string& name);

	/// Desctructor
	~RecordPose();

	/// \param fileName The filename to record the pose/time to (default is empty string)
	void setFileName(const std::string& fileName);

	/// \return The filename where the pose/time are recorded (default is empty string)
	const std::string& getFileName() const;

	/// \return True if the DeviceFilter is properly initialized, i.e. if the file exists and has open successfully.
	bool initialize() override;

	SURGSIM_CLASSNAME(SurgSim::Devices::RecordPose);

private:
	void filterInput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
		DataStructures::DataGroup* result) override;

	/// Timer to keep the recording real-time
	Framework::Timer m_timer;

	/// Cumulative time elapsed since the timer started (on creation of the instance, in ctor)
	double m_cumulativeTime;

	/// Filename where the poses will be recorded
	std::string m_fileName;

	/// Output stream to the file 'm_fileName', the entire content is replaced at each run
	std::ofstream m_outputFile;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_RECORDPOSE_H
