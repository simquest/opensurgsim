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
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | pose       | "pose"			   | %Hand pose w.r.t. JointFrameIndex.ROOT_JOINT (units are meters).		   |
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

	virtual bool initialize() override;

	virtual bool finalize() override;

	/// \return True, if this device is initialized.
	bool isInitialized() const;

	// Joint frames used for skinning.
	//
	// These are the frames stored in HandTrackingData.jointQuaternions/HandTrackingData.jointPositions.
	// They are used to locate points in 3D space or define 3D frames. If you want to recognize gestures of the form
	// "finger X is bending" it is recommended to use the FingerDOF instead. The most stable frame is the one defined
	// by the metacarpals (WRIST_JOINT).
	enum JointFrameIndex
	{
		ROOT_JOINT = 0,           // The frame of the user's forearm (this is where the hand model is rooted).
		WRIST_JOINT = 1,          // The frame of the back of the hand (anatomically, this is the frame of the carpals).
		THUMB_PROXIMAL = 2,       // Thumb proximal frame, refers to the thumb metacarpal bone.
		THUMB_INTERMEDIATE = 3,   // Thumb intermediate frame, refers to the thumb proximal phalange.
		THUMB_DISTAL = 4,         // Thumb distal frame, refers to the thumb distal phalange.
		INDEX_PROXIMAL = 5,       // Index finger proximal frame, refers to the proximal phalange.
		INDEX_INTERMEDIATE = 6,   // Index finger intermediate frame, refers to the intermediate phalange.
		INDEX_DISTAL = 7,         // Index finger distal frame, refers to the distal phalange.
		MIDDLE_PROXIMAL = 8,      // Middle finger proximal frame, refers to the proximal phalange.
		MIDDLE_INTERMEDIATE = 9,  // Middle finger intermediate frame, refers to the intermediate phalange.
		MIDDLE_DISTAL = 10,       // Middle finger distal frame, refers to the distal phalange.
		RING_PROXIMAL = 11,       // Ring finger proximal frame, refers to the proximal phalange.
		RING_INTERMEDIATE = 12,   // Ring finger intermediate frame, refers to the intermediate phalange.
		RING_DISTAL = 13,         // Ring finger distal frame, refers to the distal phalange.
		PINKY_PROXIMAL = 14,      // Pinky finger proximal frame, refers to the proximal phalange.
		PINKY_INTERMEDIATE = 15,  // Pinky finger intermediate frame, refers to the intermediate phalange.
		PINKY_DISTAL = 16,        // Pinky finger distal frame, refers to the distal phalange.
	};

	// Degrees of freedom of the hand model.
	//
	// Stored in HandTrackingData.fingerDofs. Each of these is a rotation in radians that measures how far the
	// corresponding finger is bent from its rest pose. For flexion/extension joints, positive angles indicate
	// flexion while negative angles indicate extension.
	//
	// Recommended for detecting gestures of the form "finger X is bending." To know the global position/orientation
	// of the hand, use HandTrackingData.jointPositions/HandTrackingData.jointQuaternions instead.
	enum FingerDOF
	{
		THUMB_CMC_AA  = 0,     ///< Thumb carpal-metacarpal joint, adduction/abduction
		THUMB_CMC_FE  = 1,     ///< Thumb carpal-metacarpal joint, flexion/extension
		THUMB_MCP     = 2,     ///< Thumb metacarpal-phalangeal joint, flexion/extension
		THUMB_IP      = 3,     ///< Thumb interphalangeal joint, flexion/extension
		INDEX_MCP_AA  = 4,     ///< Index finger metacarpal-phalangeal joint, adduction/abduction
		INDEX_MCP_FE  = 5,     ///< Index finger metacarpal-phalangeal joint, flexion/extension
		INDEX_PIP     = 6,     ///< Index finger proximal interphalangeal joint, flexion/extension
		MIDDLE_MCP_AA = 7,     ///< Middle finger metacarpal-phalangeal joint, adduction/abduction
		MIDDLE_MCP_FE = 8,     ///< Middle finger metacarpal-phalangeal joint, flexion/extension
		MIDDLE_PIP    = 9,     ///< Middle finger proximal interphalangeal joint, flexion/extension
		RING_MCP_AA   = 10,    ///< Ring finger metacarpal-phalangeal joint, adduction/abduction
		RING_MCP_FE   = 11,    ///< Ring finger metacarpal-phalangeal joint, flexion/extension
		RING_PIP      = 12,    ///< Ring finger proximal interphalangeal joint, flexion/extension
		PINKY_MCP_AA  = 13,    ///< Pinky finger metacarpal-phalangeal joint, adduction/abduction
		PINKY_MCP_FE  = 14,    ///< Pinky finger metacarpal-phalangeal joint, flexion/extension
		PINKY_PIP     = 15     ///< Pinky finger proximal interphalangeal joint, flexion/extension
	};

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
