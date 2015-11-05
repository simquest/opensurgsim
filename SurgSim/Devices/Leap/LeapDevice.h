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

#ifndef SURGSIM_DEVICES_LEAP_LEAPDEVICE_H
#define SURGSIM_DEVICES_LEAP_LEAPDEVICE_H

#include <memory>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Input/CommonDevice.h"


namespace SurgSim
{
namespace Devices
{

class LeapScaffold;

SURGSIM_STATIC_REGISTRATION(LeapDevice);

enum HandType : SURGSIM_ENUM_TYPE;

/// A class implementing the communication with one hand tracked by Leap Motion camera
///
/// \par Application input provided by the device:
///   | type       | name              			|                                                                   |
///   | ----       | ----              			| ---                                                               |
///   | image      | "left"						| Left infrared image, each pixel value is between 0 and 1.			|
///   | image      | "right"					| Right infrared image, each pixel value is between 0 and 1.		|
///   | image      | "left_distortion"			| Left distortion callibration map									|
///   | image      | "right_distortion"			| Right distortion callibration map									|
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
/// \sa SurgSim::Devices::LeapScaffold
class LeapDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param name A unique name for the device that will be used by the application.
	explicit LeapDevice(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Devices::LeapDevice);

	/// Destructor.
	virtual ~LeapDevice();

	/// Set the type of hand
	/// \param type The hand type, either HANDTYPE_LEFT or HANDTYPE_RIGHT
	void setHandType(HandType type);

	/// Get the type of hand
	/// \return The hand type, either HANDTYPE_LEFT or HANDTYPE_RIGHT
	HandType getHandType() const;

	/// Set the hand tracking mode to HMD
	/// This is a global setting that optimizes hand tracking based on the Leap camera
	/// placement. The default is desktop mode, where the camera is placed
	/// face up on a desktop. Use Hmd mode when the camera is attached to
	/// the front of a head mounted display.
	/// \param useHmdTrackingMode True if Hmd tracking mode is to be used
	void setUseHmdTrackingMode(bool useHmdTrackingMode);

	/// Is Using HMD Tracking Mode
	/// \return True is HMD Tracking Mode is on
	bool isUsingHmdTrackingMode() const;

	/// Set if the device should provide the stereo infrared images
	/// \param provideImages True if providing images
	void setProvideImages(bool provideImages);

	/// Get if the device should provide the stereo infrared images
	/// \return True if providing images
	bool isProvidingImages() const;

	bool initialize() override;

	bool isInitialized() const override;

private:
	friend class LeapScaffold;

	bool finalize() override;

	std::shared_ptr<LeapScaffold> m_scaffold;

	HandType m_handType;

	/// Tracking mode
	DataStructures::OptionalValue<bool> m_requestedHmdTrackingMode;

	/// Request Camera Images mode
	bool m_isProvidingImages;
};

};
};

SURGSIM_SERIALIZABLE_ENUM(SurgSim::Devices::HandType, (HANDTYPE_LEFT)(HANDTYPE_RIGHT));

#endif //SURGSIM_DEVICES_LEAP_LEAPDEVICE_H
