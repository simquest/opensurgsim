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

#include "SurgSim/Devices/MultiAxis/RawMultiAxisScaffold.h"

#include <linux/input.h>

#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>

#include <vector>
#include <list>
#include <array>
#include <memory>
#include <algorithm>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "SurgSim/Devices/MultiAxis/RawMultiAxisDevice.h"
#include "SurgSim/Devices/MultiAxis/RawMultiAxisThread.h"
#include "SurgSim/Devices/MultiAxis/FileDescriptor.h"
#include "SurgSim/Devices/MultiAxis/BitSetBuffer.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector3f;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;


namespace SurgSim
{
namespace Device
{


// Get the output location for the XSI version of strerror_r.
static inline const char* systemErrorTextHelper(const char* buffer, int returnValue)
{
	if (returnValue != 0)
	{
		return nullptr;
	}
	return buffer;
}

// Get the output location for the GNU version of strerror_r.
static inline const char* systemErrorTextHelper(const char* buffer, const char* returnValue)
{
	return returnValue;
}

static std::string getSystemErrorText(int error)
{
	char buffer[1024];
	buffer[0] = '\0';
	// Unfortunately, on Linux you can't really know if you will get the XSI version or the GNU version of
	// strerror_r.  Fortunately, the arguments are the same (only return type differs), so we can cheat.
	const char* message = systemErrorTextHelper(buffer, strerror_r(error, buffer, sizeof(buffer)));
	if (message == nullptr || message[0] == '\0')
	{
		snprintf(buffer, sizeof(buffer), "<system error %d>", error);
		message = buffer;
	}
	return std::string(message);
}


struct RawMultiAxisScaffold::DeviceData
{
public:
	/// Initialize the data.
	DeviceData(const std::string& path, RawMultiAxisDevice* device, FileDescriptor&& descriptor,
			   const std::vector<int>& buttonCodeList) :
		devicePath(path),
		deviceObject(device),
		thread(),
		fileDescriptor(std::move(descriptor)),
		axisStates(initialAxisStates()),
		buttonStates(initialButtonStates()),
		coordinateSystemRotation(defaultCoordinateSystemRotation()),
		positionScale(device->getPositionScale()),
		orientationScale(device->getOrientationScale()),
		useAxisDominance(device->isUsingAxisDominance())
	{
		for (size_t i = 0;  (i < buttonCodeList.size()) && (i < NUM_BUTTONS);  ++i)
		{
			buttonCodes[i] = buttonCodeList[i];
		}
		for (size_t i = buttonCodeList.size();  i < NUM_BUTTONS;  ++i)
		{
			buttonCodes[i] = -1;
		}
	}

	// Initialize by moving the data from another object.
	// Needed because Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers.
	DeviceData(DeviceData&& other) :
		devicePath(std::move(other.devicePath)),
		deviceObject(std::move(other.deviceObject)),
		thread(std::move(other.thread)),
		fileDescriptor(std::move(other.fileDescriptor)),
		axisStates(std::move(other.axisStates)),
		buttonStates(std::move(other.buttonStates)),
		buttonCodes(std::move(other.buttonCodes)),
		coordinateSystemRotation(std::move(other.coordinateSystemRotation)),
		positionScale(std::move(other.positionScale)),
		orientationScale(std::move(other.orientationScale)),
		useAxisDominance(std::move(other.useAxisDominance))
	{
	}

	~DeviceData()
	{
	}

	static const size_t NUM_BUTTONS = 4;

	// Returns the default coordinate system rotation matrix.
	static SurgSim::Math::Matrix33d defaultCoordinateSystemRotation()
	{
		SurgSim::Math::Matrix33d coordinateSystemRotation;
		// Make +Y point up (3DConnexion devices use +Z up)
		coordinateSystemRotation <<
			1, 0, 0,
			0, 0, -1,
			0, 1, 0;
		return coordinateSystemRotation;
	}

	static std::array<int, 6> initialAxisStates()
	{
		std::array<int, 6> zeros;
		zeros.fill(0);
		return zeros;
	}

	static std::array<bool, NUM_BUTTONS> initialButtonStates()
	{
		std::array<bool, NUM_BUTTONS> states;
		states.fill(false);
		return states;
	}

	/// The system device path corresponding to this device.
	const std::string devicePath;
	/// The corresponding device object.
	RawMultiAxisDevice* const deviceObject;
	/// Processing thread.
	std::unique_ptr<RawMultiAxisThread> thread;
	/// File descriptor to read from.
	FileDescriptor fileDescriptor;
	/// Persistent axis states.
	std::array<int, 6> axisStates;
	/// Persistent button states.
	std::array<bool, NUM_BUTTONS> buttonStates;
	/// Event library button code corresponding to each index.
	std::array<int, NUM_BUTTONS> buttonCodes;
	/// The rotation of the coordinate system (used to reorient, e.g. point +Y up)
	SurgSim::Math::Matrix33d coordinateSystemRotation;
	/// Scale factor for the position axes.
	double positionScale;
	/// Scale factor for the orientation axes.
	double orientationScale;
	/// Controls whether dominance will be enabled.
	bool useAxisDominance;
	/// The mutex that protects the externally modifiable parameters.
	boost::mutex parametersMutex;

private:
	// prohibit copy construction and asignment
	DeviceData(const DeviceData&) = delete;
	DeviceData& operator=(const DeviceData&) = delete;
};

struct RawMultiAxisScaffold::StateData
{
public:
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// The list of known devices.
	std::list<std::unique_ptr<RawMultiAxisScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// prohibit copy construction and asignment
	StateData(const StateData&) = delete;
	StateData& operator=(const StateData&) = delete;
};


RawMultiAxisScaffold::RawMultiAxisScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new StateData)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::createConsoleLogger("RawMultiAxis device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Shared scaffold created.";
}


RawMultiAxisScaffold::~RawMultiAxisScaffold()
{
	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Destroying scaffold while devices are active!?!";
			// do anything special with each device?
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Shared scaffold destroyed.";
}


bool RawMultiAxisScaffold::registerDevice(RawMultiAxisDevice* device)
{
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->isApiInitialized)
		{
			if (! initializeSdk())
			{
				return false;
			}
		}
	}

	int numUsedDevicesSeen = 0;
	bool deviceFound = findUnusedDeviceAndRegister(device, &numUsedDevicesSeen);
	if (! deviceFound)
	{
		if (numUsedDevicesSeen > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Failed to find any unused multi-axis controllers," <<
				" out of " << numUsedDevicesSeen << " present!";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Failed to find any multi-axis controllers." <<
				"  Is one plugged in?  Are the permissions correct?";
		}
		return false;
	}

	return true;
}


bool RawMultiAxisScaffold::unregisterDevice(const RawMultiAxisDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			if ((*matching)->thread)
			{
				destroyPerDeviceThread(matching->get());
			}
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (! found)
	{
		SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: Attempted to release a non-registered device.";
	}
	return found;
}

void RawMultiAxisScaffold::setPositionScale(const RawMultiAxisDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->positionScale = scale;
	}
}

void RawMultiAxisScaffold::setOrientationScale(const RawMultiAxisDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->orientationScale = scale;
	}
}

void RawMultiAxisScaffold::setAxisDominance(const RawMultiAxisDevice* device, bool onOff)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->useAxisDominance = onOff;
	}
}

bool RawMultiAxisScaffold::runInputFrame(RawMultiAxisScaffold::DeviceData* info)
{
	info->deviceObject->pullOutput();
	if (! updateDevice(info))
	{
		return false;
	}
	info->deviceObject->pushInput();
	return true;
}

static int findDominantAxis(const std::array<int, 6>& axes)
{
	int biggestAxis = 0;
	int biggestValue = abs(axes[biggestAxis]);
	for (int i = 1;  i < 6;  ++i)
	{
		int value = abs(axes[i]);
		if (value > biggestValue)
		{
			biggestValue = value;
			biggestAxis = i;
		}
		else if (value == biggestValue)
		{
			return -1;  // a tie means no dominant axis
		}
	}
	return biggestAxis;
}

bool RawMultiAxisScaffold::updateDevice(RawMultiAxisScaffold::DeviceData* info)
{
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();

	boost::lock_guard<boost::mutex> lock(info->parametersMutex);

	bool ledState = false;
	if (outputData.booleans().get("led1", &ledState))
	{
		static bool firstTimeWarning = true;
		if (firstTimeWarning)
		{
			firstTimeWarning = false;
			SURGSIM_LOG_CRITICAL(m_logger) << "RawMultiAxis: controlling LEDs is not supported yet!";
			// TODO(advornik): We should implement LED control.  But that probably doesn't mix well with blocking
			// reads here, and would either need to be done in another thread, or we'd need to poll here.
		}
	}

	struct pollfd pollData[1];
	pollData[0].fd = info->fileDescriptor.get();
	pollData[0].events = POLLIN;

	//const int timeoutMsec = 10;
	const int nonBlockingOnly = 0;
	bool didUpdate = false;
	while (poll(pollData, 1, nonBlockingOnly) > 0)
	{
		struct input_event event;
		int numRead = read(info->fileDescriptor.get(), &event, sizeof(event));
		if (numRead < 0)
		{

			if (errno == ENODEV)
			{
				SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: read failed; device has been disconnected!  (ignoring)";
				return false;  // stop updating this device!
			}
			else if (errno != EAGAIN)
			{
				SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: read failed with error " << errno << ", " <<
					getSystemErrorText(errno);
			}
		}
		else if (numRead != sizeof(event))
		{
			SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: reading produced " << numRead << " bytes (expected " <<
				sizeof(event) << ")";
		}
		else
		{
			if (event.type == EV_REL)
			{
				if (event.code >= REL_X && event.code < (REL_X+3))  // Assume that X, Y, Z are consecutive
				{
					info->axisStates[0 + (event.code - REL_X)] = event.value;
					didUpdate = true;
				}
				else if (event.code >= REL_RX && event.code < (REL_RX+3))  // Assume that RX, RY, RZ are consecutive
				{
					info->axisStates[3 + (event.code - REL_RX)] = event.value;
					didUpdate = true;
				}
			}
			else if (event.type == EV_ABS)
			{
				if (event.code >= ABS_X && event.code < (ABS_X+3))  // Assume that X, Y, Z are consecutive
				{
					info->axisStates[0 + (event.code - ABS_X)] = event.value;
					didUpdate = true;
				}
				else if (event.code >= ABS_RX && event.code < (ABS_RX+3))  // Assume that RX, RY, RZ are consecutive
				{
					info->axisStates[3 + (event.code - ABS_RX)] = event.value;
					didUpdate = true;
				}
			}
			else if (event.type == EV_KEY)
			{
				for (size_t i = 0;  i < DeviceData::NUM_BUTTONS;  ++i)
				{
					if (event.code == info->buttonCodes[i])
					{
						info->buttonStates[i] = (event.value != 0);
						didUpdate = true;
						break;
					}
				}
			}
		}
	}

	if (didUpdate)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: STATE  " <<
			std::setw(3) << info->axisStates[0] << " " <<   std::setw(3) << info->axisStates[1] << " " <<
			std::setw(3) << info->axisStates[2] << " / " << std::setw(3) << info->axisStates[3] << " " <<
			std::setw(3) << info->axisStates[4] << " " <<   std::setw(3) << info->axisStates[5] << " :" <<
			(info->buttonStates[0] ? " X" : " _") << (info->buttonStates[1] ? " X" : " _") <<
			(info->buttonStates[2] ? " X" : " _") << (info->buttonStates[3] ? " X" : " _");
	}

	// Deal with dominance and scaling.
	// It would be neat to put dominance into a filter, outside of the raw multi-axis device... but it has to
	// happen before filtering, and putting dominance AND filtering (and integrator) into components is too much.
	Vector3d position;
	Vector3d rotation;
	if (! info->useAxisDominance)
	{
		position = Vector3d(info->axisStates[0], info->axisStates[1], info->axisStates[2]) * info->positionScale;
		rotation = Vector3d(info->axisStates[3], info->axisStates[4], info->axisStates[5]) * info->orientationScale;
	}
	else
	{
		position.setZero();
		rotation.setZero();
		int index = findDominantAxis(info->axisStates);
		if (index >= 0 && index < 3)
		{
			position[index] = info->axisStates[index] * info->positionScale;
		}
		else if (index >= 3 && index < 6)
		{
			rotation[index-3] = info->axisStates[index] * info->orientationScale;
		}
	}

	// Fix up the coordinate system (3DConnexion devices use +Z up coordinates, we want +Y up).
	position = info->coordinateSystemRotation * position;
	rotation = info->coordinateSystemRotation * rotation;

	// Convert to a pose.
	Matrix33d orientation;
	double angle = rotation.norm();
	if (angle < 1e-9)
	{
		orientation.setIdentity();
	}
	else
	{
		orientation = SurgSim::Math::makeRotationMatrix(angle, Vector3d(rotation / angle));
	}

	RigidTransform3d pose;
	pose.makeAffine();
	pose.linear() = orientation;
	pose.translation() = position;

	// TODO(bert): this code should cache the access indices.
	inputData.poses().set("pose", pose);
	inputData.booleans().set("button1", info->buttonStates[0]);
	inputData.booleans().set("button2", info->buttonStates[1]);
	inputData.booleans().set("button3", info->buttonStates[2]);
	inputData.booleans().set("button4", info->buttonStates[3]);

	return true;
}

bool RawMultiAxisScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);

	// nothing to do!

	m_state->isApiInitialized = true;
	return true;
}

bool RawMultiAxisScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	// nothing to do!

	m_state->isApiInitialized = false;
	return true;
}

FileDescriptor RawMultiAxisScaffold::openDevice(const std::string& path)
{
	FileDescriptor fd;
	fd.openForReadingAndMaybeWriting(path);
	if (! fd.isValid())
	{
		int error = errno;
		if (error != ENOENT)
		{
			SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: Could not open device " << path << ": " <<
				getSystemErrorText(error);
		}
	}
	return std::move(fd);
}

bool RawMultiAxisScaffold::findUnusedDeviceAndRegister(RawMultiAxisDevice* device, int* numUsedDevicesSeen)
{
	*numUsedDevicesSeen = 0;

	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "RawMultiAxis: Tried to register a device" <<
		" which is already present!";

	// Make sure the name is unique.
	const std::string deviceName = device->getName();
	auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "RawMultiAxis: Tried to register a device when the same name is" <<
			" already present!";
		return false;
	}

	for (int i = 0;  i < 32;  ++i)
	{
		char devicePath[128];
		snprintf(devicePath, sizeof(devicePath), "/dev/input/event%d", i);

		FileDescriptor fd = openDevice(devicePath);
		if (! fd.isValid())
		{
			continue;
		}

		char reportedName[1024];
		if (ioctl(fd.get(), EVIOCGNAME(sizeof(reportedName)), reportedName) < 0)
		{
			SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGNAME): " << errno << ", " <<
				getSystemErrorText(errno);
			snprintf(reportedName, sizeof(reportedName), "???");
		}
		else
		{
			reportedName[sizeof(reportedName)-1] = '\0';
		}
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Examining device " << devicePath << " (" << reportedName << ")";

		struct input_id reportedId;
		if (ioctl(fd.get(), EVIOCGID, &reportedId) < 0)
		{
			SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGID): " << errno << ", " <<
				getSystemErrorText(errno);
			continue;
		}

		if (! deviceHasSixAbsoluteAxes(fd) && ! deviceHasSixRelativeAxes(fd))
		{
			continue;
		}

		fd.reset();

		if (registerIfUnused(devicePath, device, numUsedDevicesSeen))
		{
			return true;
		}
	}

	return false;
}

bool RawMultiAxisScaffold::registerIfUnused(const std::string& path, RawMultiAxisDevice* device,
											int* numUsedDevicesSeen)
{
	// Check existing devices.
	auto sameIndices = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[path](const std::unique_ptr<DeviceData>& info) { return (info->devicePath == path); });
	if (sameIndices != m_state->activeDeviceList.end())
	{
		// We found an existing device for this controller.
		++(*numUsedDevicesSeen);
		return false;
	}

	// The controller is not yet in use.

	FileDescriptor fd = openDevice(path);
	if (! fd.isValid())
	{
		return false;
	}

	std::vector<int> buttons = getDeviceButtonsAndKeys(fd);

	// Construct the object, start its thread, then move it to the list.
	// The thread needs a device entry pointer, but the unique_ptr indirection means we have one to provide even
	// before we've put an entry in the active device array.
	std::unique_ptr<DeviceData> info(new DeviceData(path, device, std::move(fd), buttons));
	createPerDeviceThread(info.get());
	SURGSIM_ASSERT(info->thread);
	m_state->activeDeviceList.emplace_back(std::move(info));

	return true;
}

bool RawMultiAxisScaffold::createPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(! data->thread);

	std::unique_ptr<RawMultiAxisThread> thread(new RawMultiAxisThread(this, data));
	thread->start();
	data->thread = std::move(thread);

	return true;
}

bool RawMultiAxisScaffold::destroyPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(data->thread);

	std::unique_ptr<RawMultiAxisThread> thread = std::move(data->thread);
	thread->stop();
	thread.release();

	return true;
}

bool RawMultiAxisScaffold::deviceHasSixAbsoluteAxes(const FileDescriptor& fileDescriptor)
{
	BitSetBuffer<ABS_CNT> buffer;
	if (ioctl(fileDescriptor.get(), EVIOCGBIT(EV_ABS, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_ABS)): " << errno << ", " <<
			getSystemErrorText(errno);
		return false;
	}

	if (! buffer.test(ABS_X) || ! buffer.test(ABS_Y) || ! buffer.test(ABS_Z) ||
		! buffer.test(ABS_RX) || ! buffer.test(ABS_RY) || ! buffer.test(ABS_RZ))
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: does not have the 6 absolute axes.";
		return false;
	}

	int numIgnoredAxes = 0;
	for (size_t i = 0;  i < ABS_CNT;  ++i)
	{
		if (buffer.test(i))
		{
			if ((i != ABS_X) && (i != ABS_Y) && (i != ABS_Z) && (i != ABS_RX) && (i != ABS_RY) && (i != ABS_RZ))
			{
				++numIgnoredAxes;
			}
		}
	}
	if (numIgnoredAxes)
	{
		SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: ignoring " << numIgnoredAxes << " additional absolute axes.";
	}

	return true;
}

bool RawMultiAxisScaffold::deviceHasSixRelativeAxes(const FileDescriptor& fileDescriptor)
{
	BitSetBuffer<REL_CNT> buffer;
	if (ioctl(fileDescriptor.get(), EVIOCGBIT(EV_REL, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_REL)): " << errno << ", " <<
			getSystemErrorText(errno);
		return false;
	}

	if (! buffer.test(REL_X) || ! buffer.test(REL_Y) || ! buffer.test(REL_Z) ||
		! buffer.test(REL_RX) || ! buffer.test(REL_RY) || ! buffer.test(REL_RZ))
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: does not have the 6 relative axes.";
		return false;
	}

	int numIgnoredAxes = 0;
	for (size_t i = 0;  i < REL_CNT;  ++i)
	{
		if (buffer.test(i))
		{
			if ((i != REL_X) && (i != REL_Y) && (i != REL_Z) && (i != REL_RX) && (i != REL_RY) && (i != REL_RZ))
			{
				++numIgnoredAxes;
			}
		}
	}
	if (numIgnoredAxes)
	{
		SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: ignoring " << numIgnoredAxes << " additional relative axes.";
	}

	return true;
}

std::vector<int> RawMultiAxisScaffold::getDeviceButtonsAndKeys(const FileDescriptor& fileDescriptor)
{
	std::vector<int> result;
	BitSetBuffer<KEY_CNT> buffer;
	if (ioctl(fileDescriptor.get(), EVIOCGBIT(EV_KEY, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_KEY)): " << errno << ", " <<
			getSystemErrorText(errno);
		return result;
	}

	// Start listing buttons/keys from BTN_0; then go back and cover the earlier ones.
	for (int i = BTN_0;  i < KEY_CNT;  ++i)
	{
		if (buffer.test(i))
		{
			result.push_back(i);
		}
	}
	for (int i = 0;  i < BTN_0;  ++i)
	{
		if (buffer.test(i))
		{
			result.push_back(i);
		}
	}

	return result;
}


SurgSim::DataStructures::DataGroup RawMultiAxisScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addBoolean("button1");
	builder.addBoolean("button2");
	builder.addBoolean("button3");
	builder.addBoolean("button4");
	return builder.createData();
}

std::shared_ptr<RawMultiAxisScaffold> RawMultiAxisScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<RawMultiAxisScaffold> sharedInstance;
	return sharedInstance.get();
}

void RawMultiAxisScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel RawMultiAxisScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;


};  // namespace Device
};  // namespace SurgSim
