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

#ifndef SURGSIM_DEVICES_NIMBLE_NIMBLEDEVICE_H
#define SURGSIM_DEVICES_NIMBLE_NIMBLEDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{

class NimbleScaffold;

/// A class implementing the communication with the Nimble server.
///
/// \par Application input provided by the device:
///   | type       | name						|                                                                   |
///   | ----       | ----						| ---                                                               |
///   | pose       | "pose"						| %Hand pose w.r.t. JointFrameIndex.ROOT_JOINT (units are meters).	|
///   | pose       | "ThumbProximal"			| %Pose w.r.t. thumb proximal joint									|
///   | pose       | "ThumbIntermediate"		| %Pose w.r.t. thumb intermediate joint								|
///   | pose       | "ThumbDistal"				| %Pose w.r.t. thumb distal joint									|
///   | pose       | "IndexFingerProximal"      | %Pose w.r.t. index finger proximal joint							|
///   | pose       | "IndexFingerIntermediate"	| %Pose w.r.t. index finger intermediate joint						|
///   | pose       | "IndexFingerDistal"		| %Pose w.r.t. index finger distal joint							|
///   | pose       | "MiddleFingerProximal"		| %Pose w.r.t. middle finger proximal joint							|
///   | pose       | "MiddleFingerIntermediate"	| %Pose w.r.t. middle finger intermediate joint						|
///   | pose       | "MiddleFingerDistal"		| %Pose w.r.t. middle finger distal joint							|
///   | pose       | "RingFingerProximal"		| %Pose w.r.t. ring finger proximal joint							|
///   | pose       | "RingFingerIntermediate"	| %Pose w.r.t. ring finger intermediate joint						|
///   | pose       | "RingFingerDistal"			| %Pose w.r.t. ring finger distal joint								|
///   | pose       | "SmallFingerProximal"      | %Pose w.r.t. small finger proximal joint							|
///   | pose       | "SmallFingerIntermediate"	| %Pose w.r.t. small finger intermediate joint						|
///   | pose       | "SmallFingerDistal"		| %Pose w.r.t. small finger distal joint							|
///
/// \par Application output used by the device: none.
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class NimbleDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit NimbleDevice(const std::string& uniqueName);

	/// Destructor.
	virtual ~NimbleDevice();

	/// Set the left hand to be tracked.
	void setupToTrackLeftHand();

	/// Set the right hand to be tracked.
	void setupToTrackRightHand();

	bool initialize() override;

	bool finalize() override;

	/// \return True, if this device is initialized.
	bool isInitialized() const;

private:
	friend class NimbleScaffold;

	/// The shared pointer to the NimbleScaffold.
	std::shared_ptr<NimbleScaffold> m_scaffold;

	/// Indicate whether the hand tracked is left (0) or right (1).
	size_t m_trackedHandDataIndex;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NIMBLE_NIMBLEDEVICE_H
