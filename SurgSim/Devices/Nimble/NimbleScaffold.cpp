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

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

#include "SurgSim/Devices/Nimble/NimbleScaffold.h"

#include <algorithm>
#include <list>
#include <locale>
#include <memory>
#include <vector>

#include <boost/asio.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include "SurgSim/Devices/Nimble/NimbleDevice.h"
#include "SurgSim/Devices/Nimble/NimbleThread.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Quaterniond;

namespace
{

/// Data structure to hold the data from the Nimble hand tracking SDK, for a single hand.
struct HandTrackingData
{
	/// Number of hands tracked (left, right).
	static const size_t NUM_HANDS = 2;
	/// Number of fingers tracked in one hand.
	static const size_t NUM_FINGERS = 5;
	/// Number of predefined poses for each hand.
	static const size_t NUM_POSES = 7;

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
		SMALL_PROXIMAL = 14,      // Small finger proximal frame, refers to the proximal phalange.
		SMALL_INTERMEDIATE = 15,  // Small finger intermediate frame, refers to the intermediate phalange.
		SMALL_DISTAL = 16,        // Small finger distal frame, refers to the distal phalange.
		NUM_JOINTS
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
		SMALL_MCP_AA  = 13,    ///< Small finger metacarpal-phalangeal joint, adduction/abduction
		SMALL_MCP_FE  = 14,    ///< Small finger metacarpal-phalangeal joint, flexion/extension
		SMALL_PIP     = 15,    ///< Small finger proximal interphalangeal joint, flexion/extension
		NUM_FINGER_DOFS_PER_HAND
	};

	struct HandData
	{
		/// Transform of the hand (w.r.t JointFrameIndex.ROOT_JOINT).
		RigidTransform3d pose;
		/// Number of times the hand was clicked. 0, 1 or 2.
		int clickCount;
		/// Value between 0 and 1 to specify the confidence of the poses. Currently, either 0 or 1.
		double confidenceEstimate;
		/// Transform of each of the joints.
		std::array<RigidTransform3d, NUM_JOINTS> jointPoses;
		/// Position of each of the finger tips.
		std::array<Vector3d, NUM_FINGERS> fingerTips;
		/// Value between 0 and 1 to specify the confidence of the hand being in one of the N_POSES poses. Sums to 1.
		std::array<double, NUM_POSES> handPoseConfidences;
		/// The angle of each of the finger joints.
		std::array<double, NUM_FINGER_DOFS_PER_HAND> fingerDofs;
	};

	std::array<HandData, 2> hands;
};

/// Parse the values in the stream into a Vector3d.
/// \param in The stream from where data is parsed.
/// \param [out] vector The object to which the parsed values are written to.
/// \return The stream that was sent in.
std::istream& operator>> (std::istream& in, Vector3d& vector)
{
	std::istream::sentry sentry(in);
	if (sentry)
	{
		in >> vector.x();
		in >> vector.y();
		in >> vector.z();
	}
	return in;
}

/// Parse the values in the stream into a Quaterniond.
/// \param in The stream from where data is parsed.
/// \param [out] quaternion The object to which the parsed values are written to.
/// \return The stream that was sent in.
std::istream& operator>> (std::istream& in, Quaterniond& quaternion)
{
	std::istream::sentry sentry(in);
	if (sentry)
	{
		in >> quaternion.x();
		in >> quaternion.y();
		in >> quaternion.z();
		in >> quaternion.w();
	}
	return in;
}

/// Parse the values in the stream into the HandTracking Data structure.
/// \param in The stream from where data is parsed.
/// \param [out] handData The object to which the parsed values are written to.
/// \return The stream that was sent in.
std::istream& operator>> (std::istream& in, HandTrackingData& handData)
{
	Vector3d position;
	Quaterniond quaternion;

	for (auto hand = handData.hands.begin(); in.good() && hand != handData.hands.end(); ++hand)
	{
		in >> position;
		hand->pose.translation() = position;
		in >> quaternion;
		hand->pose.linear() = quaternion.matrix();
		in >> hand->clickCount;
	}

	for (auto hand = handData.hands.begin(); in.good() && hand != handData.hands.end(); ++hand)
	{
		in >> hand->confidenceEstimate;

		for (auto jointPose = hand->jointPoses.begin(); in.good() && jointPose != hand->jointPoses.end(); ++jointPose)
		{
			in >> position;
			jointPose->translation() = position;
			in >> quaternion;
			jointPose->linear() = quaternion.matrix();
		}

		for (auto fingerTip = hand->fingerTips.begin(); in.good() && fingerTip != hand->fingerTips.end(); ++fingerTip)
		{
			in >> *fingerTip;
		}
	}

	for (auto hand = handData.hands.begin(); in.good() && hand != handData.hands.end(); ++hand)
	{
		for (auto handPoseConfidence = hand->handPoseConfidences.begin(); in.good() &&
			 handPoseConfidence != hand->handPoseConfidences.end(); ++handPoseConfidence)
		{
			in >> *handPoseConfidence;
		}
	}

	for (auto hand = handData.hands.begin(); in.good() && hand != handData.hands.end(); ++hand)
	{
		for (auto fingerDof = hand->fingerDofs.begin(); in.good() && fingerDof != hand->fingerDofs.end(); ++fingerDof)
		{
			in >> *fingerDof;
		}
	}

	return in;
}

}

namespace SurgSim
{
namespace Device
{

std::array<std::pair<std::string, int>, 15> NimbleScaffold::m_jointPoseNames =
{
	std::make_pair("ThumbProximal", HandTrackingData::THUMB_PROXIMAL),
	std::make_pair("ThumbIntermediate", HandTrackingData::THUMB_INTERMEDIATE),
	std::make_pair("ThumbDistal", HandTrackingData::THUMB_DISTAL),
	std::make_pair("IndexFingerProximal", HandTrackingData::INDEX_PROXIMAL),
	std::make_pair("IndexFingerIntermediate", HandTrackingData::INDEX_INTERMEDIATE),
	std::make_pair("IndexFingerDistal", HandTrackingData::INDEX_DISTAL),
	std::make_pair("MiddleFingerProximal", HandTrackingData::MIDDLE_PROXIMAL),
	std::make_pair("MiddleFingerIntermediate", HandTrackingData::MIDDLE_INTERMEDIATE),
	std::make_pair("MiddleFingerDistal", HandTrackingData::MIDDLE_DISTAL),
	std::make_pair("RingFingerProximal", HandTrackingData::RING_PROXIMAL),
	std::make_pair("RingFingerIntermediate", HandTrackingData::RING_INTERMEDIATE),
	std::make_pair("RingFingerDistal", HandTrackingData::RING_DISTAL),
	std::make_pair("SmallFingerProximal", HandTrackingData::SMALL_PROXIMAL),
	std::make_pair("SmallFingerIntermediate", HandTrackingData::SMALL_INTERMEDIATE),
	std::make_pair("SmallFingerDistal", HandTrackingData::SMALL_DISTAL)
};

struct NimbleScaffold::StateData
{
public:
	/// Initialize the state.
	StateData()
	{
	}

	/// Processing thread.
	std::unique_ptr<NimbleThread> thread;

	/// The socket used for connecting to the Nimble server.
	boost::asio::ip::tcp::iostream socketStream;

	/// The hand tracking data.
	HandTrackingData handData;

	/// The list of active devices.
	std::vector<NimbleDevice*> activeDevices;

	/// The mutex that protects the active device.
	boost::mutex mutex;

private:
	// Prohibit copy construction and assignment
	StateData(const StateData&);
	StateData& operator=(const StateData&);
};

NimbleScaffold::NimbleScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new StateData()), m_serverIpAddress("127.0.0.1"), m_serverPort("1988"),
	m_serverSocketOpen(false)
{
	if (m_logger == nullptr)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("Nimble device");
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Nimble: Shared scaffold created.";
}

NimbleScaffold::~NimbleScaffold()
{
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (m_state->activeDevices.size() > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Destroying scaffold while devices are active!?!";
		}
	}

	if (m_state->thread)
	{
		destroyThread();
	}

	SURGSIM_LOG_DEBUG(m_logger) << "Nimble: Shared scaffold destroyed.";
}

bool NimbleScaffold::registerDevice(NimbleDevice* device)
{
	bool success = true;

	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		auto found = std::find_if(m_state->activeDevices.begin(), m_state->activeDevices.end(),
			[device](const NimbleDevice* it) { return it->getName() == device->getName(); });

		if (found == m_state->activeDevices.end())
		{
			m_state->activeDevices.push_back(device);
			SURGSIM_LOG_INFO(m_logger) << "Nimble: Device registered in Scaffold.";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Attempt to register device with the same name again.";
			success = false;
		}
	}

	if (success && !m_state->thread)
	{
		createThread();
	}

	return success;
}

bool NimbleScaffold::unregisterDevice(const NimbleDevice* device)
{
	bool success = true;

	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		auto found = std::find(m_state->activeDevices.begin(), m_state->activeDevices.end(), device);

		if (found != m_state->activeDevices.end())
		{
			m_state->activeDevices.erase(found);
			SURGSIM_LOG_INFO(m_logger) << "Nimble: Device unregistered from Scaffold.";
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger)
				<< "Nimble: Attempted to unregister a device from Scaffold which is not registered.";
			success = false;
		}
	}

	if (m_state->activeDevices.size() == 0 && m_state->thread)
	{
		destroyThread();
	}

	return success;
}

bool NimbleScaffold::initialize()
{
	// Connect to the Nimble hand tracking server.
	m_serverSocketOpen = true;

	m_state->socketStream.connect(m_serverIpAddress, m_serverPort);

	if (!m_state->socketStream)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Error while opening a iostream to the server: "
			<< m_state->socketStream.error().message() << ")";
		m_serverSocketOpen = false;
	}

	return m_serverSocketOpen;
}

bool NimbleScaffold::update()
{
	bool success = true;

	if (!m_state->socketStream)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Socket stream no longer good: "
			<< m_state->socketStream.error().message() << ")";
		success = false;
	}
	else
	{
		std::string messageType;
		m_state->socketStream >> messageType;

		if (messageType == "POSE")
		{
			m_state->socketStream >> m_state->handData;
			if (!m_state->socketStream.fail())
			{
				updateDeviceData();
			}
			else
			{
				SURGSIM_LOG_WARNING(m_logger)
					<< "Nimble: Hand data not parsed correctly.";
				resetDeviceData();
			}
		}
		else
		{
			m_state->socketStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
	}

	return success;
}

void NimbleScaffold::finalize()
{
	// The m_state->thread would be killed soon, so the socket is closed here.
	if (m_serverSocketOpen)
	{
		m_state->socketStream.close();
		if (m_state->socketStream.fail())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Error when shutting down socket: "
				<< m_state->socketStream.error().message() << ")";
		}
	}
}

void NimbleScaffold::updateDeviceData()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->activeDevices.begin(); it != m_state->activeDevices.end(); ++it)
	{
		size_t index = (*it)->m_trackedHandDataIndex;

		SurgSim::DataStructures::DataGroup& inputData = (*it)->getInputData();
		inputData.poses().set(SurgSim::DataStructures::Names::POSE, m_state->handData.hands[index].pose);
		for (auto name = m_jointPoseNames.begin(); name != m_jointPoseNames.end(); ++name)
		{
			inputData.poses().set(name->first, m_state->handData.hands[index].jointPoses[name->second]);
		}
		(*it)->pushInput();
	}
}

void NimbleScaffold::resetDeviceData()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->activeDevices.begin(); it != m_state->activeDevices.end(); ++it)
	{
		SurgSim::DataStructures::DataGroup& inputData = (*it)->getInputData();
		inputData.resetAll();
	}
}

bool NimbleScaffold::createThread()
{
	SURGSIM_ASSERT(!m_state->thread) << "Nimble: Attempt to create a thread when there is already one.";

	std::unique_ptr<NimbleThread> thread(new NimbleThread(this));
	thread->start();
	m_state->thread = std::move(thread);

	return true;
}

bool NimbleScaffold::destroyThread()
{
	SURGSIM_ASSERT(m_state->thread) << "Nimble: Attempt to destroy thread when there is none.";

	std::unique_ptr<NimbleThread> thread = std::move(m_state->thread);
	thread->stop();
	thread.release();

	return true;
}

SurgSim::DataStructures::DataGroup NimbleScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	for (auto name = m_jointPoseNames.begin(); name != m_jointPoseNames.end(); ++name)
	{
		builder.addPose(name->first);
	}
	return builder.createData();
}

std::shared_ptr<SurgSim::Framework::Logger> NimbleScaffold::getLogger() const
{
	return m_logger;
}

std::shared_ptr<NimbleScaffold> NimbleScaffold::getOrCreateSharedInstance()
{
	// Using an explicit creation function gets around problems with accessing the private constructor.
	static auto creator =
		[]() { return std::shared_ptr<NimbleScaffold>(new NimbleScaffold()); };  // NOLINT(readability/braces)
	static SurgSim::Framework::SharedInstance<NimbleScaffold> sharedInstance(creator);
	return sharedInstance.get();
}

}  // namespace Device
}  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
