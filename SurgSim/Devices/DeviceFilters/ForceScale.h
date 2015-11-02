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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_FORCESCALE_H
#define SURGSIM_DEVICES_DEVICEFILTERS_FORCESCALE_H

#include <boost/thread/mutex.hpp>
#include <string>

#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_STATIC_REGISTRATION(ForceScale);

/// An output device filter that scales forces and/or torques.  Any other entries in the DataGroup are passed through.
/// For convenience, it is also an InputConsumerInterface that does no filtering of the input data.  Thus it can be
/// added as an input consumer to the raw device, and set as the output producer for the raw device, after which other
/// device filters, input components, and output components only need access to the ForceScale instance, not the raw
/// device.
class ForceScale : public DeviceFilter
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit ForceScale(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Devices::ForceScale);

	/// Set the force scale factor so that each direction has the same scale.
	/// \param forceScale The scalar scaling factor.
	void setForceScale(double forceScale);

	/// Set the torque scale factor so that each direction has the same scale.
	/// \param torqueScale The scalar scaling factor.
	void setTorqueScale(double torqueScale);

private:
	void filterOutput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
		DataStructures::DataGroup* result) override;

	/// The mutex that protects the scaling factors.
	boost::mutex m_mutex;

	/// The scaling factor applied to each direction of the force.
	double m_forceScale;

	/// The scaling factor applied to each direction of the torque.
	double m_torqueScale;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_FORCESCALE_H
