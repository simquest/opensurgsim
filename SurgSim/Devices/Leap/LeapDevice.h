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

#ifndef SURGSIM_DEVICES_LEAP_LEAPDEVICE_H
#define SURGSIM_DEVICES_LEAP_LEAPDEVICE_H

#include <memory>

#include "SurgSim/Input/CommonDevice.h"


namespace SurgSim
{
namespace Device
{

class LeapScaffold;

enum HandType
{
	HANDTYPE_LEFT,
	HANDTYPE_RIGHT
};

enum LeapTrackingMode
{
	LEAP_TRACKING_MODE_DESKTOP,
	LEAP_TRACKING_MODE_HMD
};

/// A class implementing the communication with one hand tracked by Leap Motion camera
///
/// \par Application input provided by the device:
///   | type       | name              			|                                                                   |
///   | ----       | ----              			| ---                                                               |
///   | pose       | "pose"						| %Hand pose 														|
///   | pose       | "ThumbProximal"			| %Pose of thumb proximal joint										|
///   | pose       | "ThumbIntermediate"		| %Pose of thumb intermediate joint									|
///   | pose       | "ThumbDistal"				| %Pose of thumb distal joint										|
///   | pose       | "IndexFingerProximal"      | %Pose of index finger proximal joint								|
///   | pose       | "IndexFingerIntermediate"	| %Pose of index finger intermediate joint							|
///   | pose       | "IndexFingerDistal"		| %Pose of index finger distal joint								|
///   | pose       | "MiddleFingerProximal"		| %Pose of middle finger proximal joint								|
///   | pose       | "MiddleFingerIntermediate"	| %Pose of middle finger intermediate joint							|
///   | pose       | "MiddleFingerDistal"		| %Pose of middle finger distal joint								|
///   | pose       | "RingFingerProximal"		| %Pose of ring finger proximal joint								|
///   | pose       | "RingFingerIntermediate"	| %Pose of ring finger intermediate joint							|
///   | pose       | "RingFingerDistal"			| %Pose of ring finger distal joint									|
///   | pose       | "SmallFingerProximal"      | %Pose of small finger proximal joint								|
///   | pose       | "SmallFingerIntermediate"	| %Pose of small finger intermediate joint							|
///   | pose       | "SmallFingerDistal"		| %Pose of small finger distal joint								|
///
/// \par Application output used by the device: none.
///
/// \sa SurgSim::Device::LeapScaffold
class LeapDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param name A unique name for the device that will be used by the application.
	explicit LeapDevice(const std::string& name);

	/// Destructor.
	virtual ~LeapDevice();

	/// Set the type of hand, left or right
	/// \param type The hand type, either HANDTYPE_LEFT or HANDTYPE_RIGHT
	void setHandType(HandType type);

	/// Get the type of hand, left or right
	/// \return The hand type, either HANDTYPE_LEFT or HANDTYPE_RIGHT
	HandType getHandType() const;

	/// Set tracking mode: LEAP_TRACKING_MODE_DESKTOP or LEAP_TRACKING_MODE_HMD
	/// \param mode tracking mode
	void setTrackingMode(LeapTrackingMode mode);

	/// Get tracking mode: LEAP_TRACKING_MODE_DESKTOP or LEAP_TRACKING_MODE_HMD
	/// \return current tracking mode
	LeapTrackingMode getTrackingMode() const;

	bool initialize() override;

	bool finalize() override;

	/// Check if this device is initialized.
	/// \return True if this device is initialized; otherwise, false.
	bool isInitialized() const;

private:
	friend class LeapScaffold;

	std::shared_ptr<LeapScaffold> m_scaffold;

	HandType m_handType;

	/// Tracking mode
	LeapTrackingMode m_trackingMode;
};

};
};

#endif //SURGSIM_DEVICES_LEAP_LEAPDEVICE_H
