// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DEVICES_LEAP_LEAPUTILITIES_H
#define SURGSIM_DEVICES_LEAP_LEAPUTILITIES_H

#include "SurgSim/DataStructures/DataGroup.h"


namespace SurgSim
{
namespace Devices
{

/// Correct an infrared image from the LeapDevice, given its distortion callibration map
/// \param image The distorted infrared image
/// \param distortion The distortion callibration map
/// \return The corrected infrared image
DataStructures::DataGroup::ImageType undistortLeapImage(const DataStructures::DataGroup::ImageType& image,
		const DataStructures::DataGroup::ImageType& distortion);

};
};

#endif //SURGSIM_DEVICES_LEAP_LEAPUTILITIES_H

