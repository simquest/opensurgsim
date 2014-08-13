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
	static const size_t N_HANDS = 2;
	/// Number of joints in one hand.
	static const size_t N_JOINTS = 17;
	/// Number of fingers tracked in one hand.
	static const size_t N_FINGERS = 5;
	/// Number of predefined poses for each hand.
	static const size_t N_POSES = 7;
	/// Number of DOFs for each finger.
	static const size_t N_FINGER_DOFS_PER_HAND = 16;

	/// Position of the hand.
	Vector3d position;
	/// Orientation of the hand.
	Quaterniond quaternion;
	/// Number of times the hand was clicked. 0, 1 or 2.
	int clickCount;
	/// Value between 0 and 1 to specify the confidence of the poses. Currently, either 0 or 1.
	double confidenceEstimate;
	/// Orientation of each of the joints.
	Quaterniond jointQuaternions[N_JOINTS];
	/// Position of each of the joints.
	Vector3d jointPositions[N_JOINTS];
	/// Position of each of the finger tips.
	Vector3d fingerTips[N_FINGERS];
	/// Value between 0 and 1 to specify the confidence of the hand being in one of the N_POSES poses. Sums to 1.
	double handPoseConfidences[N_POSES];
	/// The angle of each of the finger joints.
	double fingerDofs[N_FINGER_DOFS_PER_HAND];
};

/// A class to parse std::stringstream into requested data type.
/// The class is initialized with a std::stringstream and can parse it into requested datatype.
class TokenStream
{
public:
	/// Constructor.
	/// \param stream The string stream from which the values are parsed.
	explicit TokenStream(std::stringstream* stream)
		: m_stream(*stream)
	{}

	/// \param [out] var Parse stream into this int.
	/// \return True, if parsing is successful.
	bool parse(int* var)
	{
		return (m_stream >> *var).good();
	}

	/// \param [out] var Parse stream into this string.
	/// \return True, if parsing is successful.
	bool parse(std::string* var)
	{
		return (m_stream >> *var).good();
	}

	/// \param [out] var Parse stream into this double.
	/// \return True, if parsing is successful.
	bool parse(double* var)
	{
		return (m_stream >> *var).good();
	}

	/// \param [out] v Parse stream into this variable size Eigen vector.
	/// \return True, if parsing is successful.
	template <typename T, int Rows>
	bool parse(Eigen::Matrix<T, Rows, 1>* v)
	{
		bool success = true;
		for (int i = 0; success && i < Rows; ++i)
		{
			success &= parse(&(*v)[i]);
		}
		return success;
	}

	/// \param [out] q Parse stream into this variable size Eigen quaternion.
	/// \return True, if parsing is successful.
	template <typename T>
	bool parse(Eigen::Quaternion<T>* q)
	{
		return parse(&q->x()) && parse(&q->y()) && parse(&q->z()) && parse(&q->w());
	}

	/// Parse the values in the stream into the HandTracking Data structure.
	/// \param [out] handData The data structure where the parsed values are written to.
	/// \return True, if parsing is successful.
	bool parsePose(HandTrackingData* handData)
	{
		bool success = true;

		for (size_t i = 0; success && i < HandTrackingData::N_HANDS; ++i)
		{
			success &= parse(&handData[i].position);
			success &= parse(&handData[i].quaternion);
			success &= parse(&handData[i].clickCount);
		}

		for (size_t i = 0; success && i < HandTrackingData::N_HANDS; ++i)
		{
			success &= parse(&handData[i].confidenceEstimate);
			success &= handData[i].confidenceEstimate >= 0.0 && handData[i].confidenceEstimate <= 1.0;

			for (size_t j = 0; success && j < HandTrackingData::N_JOINTS; ++j)
			{
				success &= parse(&handData[i].jointQuaternions[j]);
				success &= parse(&handData[i].jointPositions[j]);
			}

			for (size_t j = 0; success && j < HandTrackingData::N_FINGERS; ++j)
			{
				success &= parse(&handData[i].fingerTips[j]);
			}
		}

		for (size_t i = 0; success && i < HandTrackingData::N_HANDS; ++i)
		{
			for (size_t j = 0; success && j < HandTrackingData::N_POSES; ++j)
			{
				success &= parse(&handData[i].handPoseConfidences[j]);
			}
		}

		for (size_t i = 0; success && i < HandTrackingData::N_HANDS; ++i)
		{
			for (size_t j = 0; success && j < HandTrackingData::N_FINGER_DOFS_PER_HAND; ++j)
			{
				success &= parse(&handData[i].fingerDofs[j]);
			}
		}

		return success;
	}

private:
	/// The string stream from where the values are parsed.
	std::stringstream& m_stream;
};

/// Parse the values in the stream based on its message type (the first few characters).
/// Only certain message types are parsed (POSE).
/// The rest of them are for other uses (CALIBRATION, PINCHING, WELCOME, USER).
/// \param stream The stream read from the socket.
/// \param [out] handData The data structure where the parsed values are written to.
/// \param [out] parseSuccess True, if the parsing to handData was a success.
/// \return True if the message was parsed, instead of being ignored.
bool processNimbleMessage(std::stringstream* stream, HandTrackingData* handData, bool* parseSuccess)
{
	bool messageParsed = false;
	TokenStream tokenStream(stream);
	std::string messageType;

	if (tokenStream.parse(&messageType) && messageType == "POSE")
	{
		*parseSuccess = tokenStream.parsePose(handData);
		messageParsed = true;
	}

	return messageParsed;
}

}

namespace SurgSim
{
namespace Device
{

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
	std::shared_ptr<boost::asio::ip::tcp::socket> socket;

	/// The hand tracking data.
	HandTrackingData handData[2];

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
	if (!m_logger)
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

	boost::asio::io_service *ioService = new boost::asio::io_service();
	boost::asio::ip::tcp::resolver resolver(*ioService);
	boost::asio::ip::tcp::resolver::query query(m_serverIpAddress, m_serverPort);
	boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);

	m_state->socket = std::make_shared<boost::asio::ip::tcp::socket>(*ioService);
	boost::system::error_code error;

	boost::asio::connect(*m_state->socket, iterator, error);

	if (!m_state->socket->is_open())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Unable to open a socket to the server.";
		m_serverSocketOpen = false;
	}

	if (error.value() > 0)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Error while opening a socket to the server: "
			<< error.value() << " (" << error.message() << ")";
		m_serverSocketOpen = false;
	}

	if (m_serverSocketOpen)
	{
		SURGSIM_LOG_INFO(m_logger) << "Nimble: Socket opened and connection established.";
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Failed to open the socket and establish connection.";
	}

	return m_serverSocketOpen;
}

bool NimbleScaffold::update()
{
	bool success = true;

	const size_t BUFFER_LENGTH = 512;
	std::array<char, BUFFER_LENGTH> buffer;

	boost::system::error_code error;
	size_t bytesRead = m_state->socket->read_some(boost::asio::buffer(buffer), error);

	// Each message from the Nimble server is terminated with a '\n' character. Although we read BUFFER_LENGTH
	// chars every time, the message can be of arbitrary length. The loop below reads BUFFER_LENGTH char at a time,
	// and keeps track of the data until a '\n' is encountered. At which point, the data is sent to
	// processNimbleMessage for parsing.
	if (bytesRead == 0)
	{
		SURGSIM_LOG_WARNING(m_logger) << "Nimble: Nothing read from server.";
	}

	if (error.value() > 0)
	{
		SURGSIM_LOG_WARNING(m_logger) << "Nimble: Error while reading from server: "
			<< error.value() << " (" << error.message() << ")";
		success = false;
	}

	if (success)
	{
		bool handDataParsed = false;
		auto bufferIterator = buffer.begin();
		for (size_t i = 0; i < bytesRead; ++i)
		{
			if (*bufferIterator != '\n')
			{
				m_serverMessage << *bufferIterator;
			}
			else
			{
				m_serverMessage << '\0';

				if (processNimbleMessage(&m_serverMessage, m_state->handData, &handDataParsed))
				{
					if (handDataParsed)
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

				m_serverMessage.str(std::string());
				m_serverMessage.clear();
			}
			++bufferIterator;
		}
	}
	else
	{
		resetDeviceData();
	}

	return success;
}

void NimbleScaffold::finalize()
{
	// The m_state->thread would be killed soon, so the socket is closed here.
	if (m_serverSocketOpen)
	{
		boost::system::error_code errorShutdown;
		m_state->socket->shutdown(boost::asio::ip::tcp::socket::socket_base::shutdown_both, errorShutdown);
		if (errorShutdown.value() > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Error when shutting down socket: "
				<< errorShutdown.value() << " (" << errorShutdown.message() << ")";
		}

		boost::system::error_code errorClose;
		m_state->socket->close(errorClose);
		if (errorClose.value() > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Nimble: Error when closing socket: "
				<< errorClose.value() << " (" << errorClose.message() << ")";
		}
	}
}

void NimbleScaffold::updateDeviceData()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->activeDevices.begin(); it != m_state->activeDevices.end(); ++it)
	{
		size_t index = (*it)->m_trackedHandDataIndex;

		RigidTransform3d pose = SurgSim::Math::makeRigidTransform(m_state->handData[index].quaternion,
									m_state->handData[index].position * (*it)->m_positionScale);

		SurgSim::DataStructures::DataGroup& inputData = (*it)->getInputData();
		inputData.poses().set(SurgSim::DataStructures::Names::POSE, pose);
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