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

#include "SurgSim/Devices/MultiAxis/linux/InputDeviceHandle.h"

#include <linux/input.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "SurgSim/Devices/MultiAxis/GetSystemError.h"
#include "SurgSim/Devices/MultiAxis/BitSetBuffer.h"
#include "SurgSim/Devices/MultiAxis/linux/FileDescriptor.h"
#include "SurgSim/Framework/Log.h"

using SurgSim::Devices::Internal::getSystemErrorCode;
using SurgSim::Devices::Internal::getSystemErrorText;


namespace SurgSim
{
namespace Devices
{

struct InputDeviceHandle::State
{
public:
	explicit State(std::shared_ptr<SurgSim::Framework::Logger>&& logger_) :
		logger(std::move(logger_)),
		handle()
	{
		buttonCodes.fill(-1);
	}

	/// The logger to use.
	std::shared_ptr<SurgSim::Framework::Logger> logger;
	/// The underlying device file descriptor.
	FileDescriptor handle;
	/// Event library button code corresponding to each index.
	std::array<int, SystemInputDeviceHandle::MAX_NUM_BUTTONS> buttonCodes;

private:
	// Prevent copy construction and copy assignment.
	State(const State& other) = delete;
	State& operator=(const State& other) = delete;
};

InputDeviceHandle::InputDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger) :
	m_state(new InputDeviceHandle::State(std::move(logger)))
{
}

InputDeviceHandle::~InputDeviceHandle()
{
}

std::vector<std::string> InputDeviceHandle::enumeratePaths(SurgSim::Framework::Logger* logger)
{
	std::vector<std::string> results;

	for (int i = 0;  i < 100;  ++i)
	{
		char devicePath[128];
		snprintf(devicePath, sizeof(devicePath), "/dev/input/event%d", i);

		FileDescriptor handle;
		if (! handle.openForReadingAndMaybeWriting(devicePath))
		{
			int error = errno;
			if (error != ENOENT)
			{
				SURGSIM_LOG_INFO(logger) << "InputDeviceHandle::enumeratePaths: Could not open device " << devicePath <<
					": error " << error << ", " << getSystemErrorText(error);
			}
			continue;
		}

		results.push_back(devicePath);
	}

	return results;
}

std::unique_ptr<InputDeviceHandle> InputDeviceHandle::open(
	const std::string& path, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	std::unique_ptr<InputDeviceHandle> object(new InputDeviceHandle(std::move(logger)));
	if (! object->m_state->handle.openForReadingAndMaybeWriting(path))
	{
		object.reset();  // could not open the device handle; destroy the object again
	}
	else
	{
		const std::vector<int> buttonCodeList = object->getDeviceButtonsAndKeys();

		for (size_t i = 0;  (i < object->m_state->buttonCodes.size()) && (i < buttonCodeList.size());  ++i)
		{
			object->m_state->buttonCodes[i] = buttonCodeList[i];
		}
		for (size_t i = buttonCodeList.size();  i < object->m_state->buttonCodes.size();  ++i)
		{
			object->m_state->buttonCodes[i] = -1;
		}
	}
	return object;
}

std::string InputDeviceHandle::getDeviceName() const
{
	char reportedName[1024];
	if (ioctl(m_state->handle.get(), EVIOCGNAME(sizeof(reportedName)), reportedName) < 0)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: ioctl(EVIOCGNAME): error " << error << ", " <<
			getSystemErrorText(error);
		snprintf(reportedName, sizeof(reportedName), "???");
	}
	else
	{
		reportedName[sizeof(reportedName)-1] = '\0';
	}
	return std::string(reportedName);
}

bool InputDeviceHandle::getDeviceIds(int* vendorId, int* productId) const
{
	struct input_id reportedId;
	if (ioctl(m_state->handle.get(), EVIOCGID, &reportedId) < 0)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: ioctl(EVIOCGID): error " << error << ", " <<
			getSystemErrorText(error);
		*vendorId = *productId = -1;
		return false;
	}

	*vendorId = reportedId.vendor;
	*productId = reportedId.product;
	return true;
}

bool InputDeviceHandle::hasAbsoluteTranslationAndRotationAxes() const
{
	BitSetBuffer<ABS_CNT> buffer;
	if (ioctl(m_state->handle.get(), EVIOCGBIT(EV_ABS, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: ioctl(EVIOCGBIT(EV_ABS)): error " <<
			error << ", " << getSystemErrorText(error);
		return false;
	}

	if (! buffer.test(ABS_X) || ! buffer.test(ABS_Y) || ! buffer.test(ABS_Z) ||
		! buffer.test(ABS_RX) || ! buffer.test(ABS_RY) || ! buffer.test(ABS_RZ))
	{
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: does not have the 6 absolute axes.";
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
		SURGSIM_LOG_INFO(m_state->logger) << "InputDeviceHandle: has absolute translation and rotation axes;" <<
			" ignoring " << numIgnoredAxes << " additional axes.";
	}

	return true;
}

bool InputDeviceHandle::hasRelativeTranslationAndRotationAxes() const
{
	BitSetBuffer<REL_CNT> buffer;
	if (ioctl(m_state->handle.get(), EVIOCGBIT(EV_REL, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: ioctl(EVIOCGBIT(EV_REL)): error " <<
			error << ", " << getSystemErrorText(error);
		return false;
	}

	if (! buffer.test(REL_X) || ! buffer.test(REL_Y) || ! buffer.test(REL_Z) ||
		! buffer.test(REL_RX) || ! buffer.test(REL_RY) || ! buffer.test(REL_RZ))
	{
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: does not have the 6 relative axes.";
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
		SURGSIM_LOG_INFO(m_state->logger) << "InputDeviceHandle: has relative translation and rotation axes;" <<
			" ignoring " << numIgnoredAxes << " additional axes.";
	}

	return true;
}

bool InputDeviceHandle::hasTranslationAndRotationAxes() const
{
	return hasAbsoluteTranslationAndRotationAxes() || hasRelativeTranslationAndRotationAxes();
}

bool InputDeviceHandle::updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated)
{
	*updated = false;

	while (m_state->handle.hasDataToRead())
	{
		struct input_event event;
		size_t numRead;
		if (! m_state->handle.readBytes(&event, sizeof(event), &numRead))
		{
			int64_t error = getSystemErrorCode();
			if (error == ENODEV)
			{
				SURGSIM_LOG_SEVERE(m_state->logger) <<
					"InputDeviceHandle: read failed; device has been disconnected!  (stopping)";
				return false;  // stop updating this device!
			}
			else
			{
				SURGSIM_LOG_WARNING(m_state->logger) << "InputDeviceHandle: read failed with error " <<
					error << ", " << getSystemErrorText(error);
			}
		}
		else if (numRead != sizeof(event))
		{
			SURGSIM_LOG_WARNING(m_state->logger) << "InputDeviceHandle: reading produced " << numRead <<
				" bytes (expected " << sizeof(event) << ")";
		}
		else
		{
			if (event.type == EV_REL)
			{

				if (event.code >= REL_X && event.code < (REL_X+3))  // Assume that X, Y, Z are consecutive
				{
					(*axisStates)[0 + (event.code - REL_X)] = event.value;
					*updated = true;
				}
				else if (event.code >= REL_RX && event.code < (REL_RX+3))  // Assume that RX, RY, RZ are consecutive
				{
					(*axisStates)[3 + (event.code - REL_RX)] = event.value;
					*updated = true;
				}
			}
			else if (event.type == EV_ABS)
			{
				if (event.code >= ABS_X && event.code < (ABS_X+3))  // Assume that X, Y, Z are consecutive
				{
					(*axisStates)[0 + (event.code - ABS_X)] = event.value;
					*updated = true;
				}
				else if (event.code >= ABS_RX && event.code < (ABS_RX+3))  // Assume that RX, RY, RZ are consecutive
				{
					(*axisStates)[3 + (event.code - ABS_RX)] = event.value;
					*updated = true;
				}
			}
			else if (event.type == EV_KEY)
			{
				for (size_t i = 0;  i < m_state->buttonCodes.size();  ++i)
				{
					if (event.code == m_state->buttonCodes[i])
					{
						(*buttonStates)[i] = (event.value != 0);
						*updated = true;
						break;
					}
				}
			}
		}
	}
	return true;
}

std::vector<int> InputDeviceHandle::getDeviceButtonsAndKeys()
{
	std::vector<int> result;
	BitSetBuffer<KEY_CNT> buffer;
	if (ioctl(m_state->handle.get(), EVIOCGBIT(EV_KEY, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_state->logger) << "InputDeviceHandle: ioctl(EVIOCGBIT(EV_KEY)): error " <<
			error << ", " << getSystemErrorText(error);
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

};  // namespace Devices
};  // namespace SurgSim
