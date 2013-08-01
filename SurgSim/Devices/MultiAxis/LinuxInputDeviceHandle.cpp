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

#include "SurgSim/Devices/MultiAxis/LinuxInputDeviceHandle.h"

#include <linux/input.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "SurgSim/Devices/MultiAxis/FileDescriptor.h"
#include "SurgSim/Devices/MultiAxis/GetSystemError.h"
#include "SurgSim/Devices/MultiAxis/BitSetBuffer.h"
#include <SurgSim/Framework/Log.h>

using SurgSim::Device::Internal::getSystemErrorCode;
using SurgSim::Device::Internal::getSystemErrorText;


namespace SurgSim
{
namespace Device
{

struct LinuxInputDeviceHandle::State
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
};

LinuxInputDeviceHandle::LinuxInputDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger) :
	m_state(new LinuxInputDeviceHandle::State(std::move(logger)))
{
}

LinuxInputDeviceHandle::~LinuxInputDeviceHandle()
{
}

std::vector<std::string> LinuxInputDeviceHandle::enumerate(SurgSim::Framework::Logger* logger)
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
				SURGSIM_LOG_INFO(logger) << "LinuxInputDeviceHandle::enumerate: Could not open device " << devicePath <<
					": error " << error << ", " << getSystemErrorText(error);
			}
			continue;
		}

		results.push_back(devicePath);
	}

	return results;
}

std::unique_ptr<LinuxInputDeviceHandle> LinuxInputDeviceHandle::open(
	const std::string& path, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	std::unique_ptr<LinuxInputDeviceHandle> object(new LinuxInputDeviceHandle(std::move(logger)));
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

bool LinuxInputDeviceHandle::canRead() const
{
	return m_state->handle.canRead();
}

bool LinuxInputDeviceHandle::canWrite() const
{
	return m_state->handle.canWrite();
}

bool LinuxInputDeviceHandle::hasDataToRead() const
{
	return m_state->handle.hasDataToRead();
}

bool LinuxInputDeviceHandle::readBytes(void* dataBuffer, size_t bytesToRead, size_t* bytesActuallyRead)
{
	return m_state->handle.readBytes(dataBuffer, bytesToRead, bytesActuallyRead);
}

// XXX HORRIBLE HACK!!!
int LinuxInputDeviceHandle::get() const
{
	return m_state->handle.get();
}

bool LinuxInputDeviceHandle::updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated)
{
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
					"RawMultiAxis: read failed; device has been disconnected!  (stopping)";
				return false;  // stop updating this device!
			}
			else if (error != EAGAIN)
			{
				SURGSIM_LOG_WARNING(m_state->logger) << "RawMultiAxis: read failed with error " << error << ", " <<
					getSystemErrorText(error);
			}
		}
		else if (numRead != sizeof(event))
		{
			SURGSIM_LOG_WARNING(m_state->logger) << "RawMultiAxis: reading produced " << numRead <<
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

std::vector<int> LinuxInputDeviceHandle::getDeviceButtonsAndKeys()
{
	std::vector<int> result;
	BitSetBuffer<KEY_CNT> buffer;
	if (ioctl(m_state->handle.get(), EVIOCGBIT(EV_KEY, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_state->logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_KEY)): error " << error << ", " <<
			getSystemErrorText(error);
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

};  // namespace Device
};  // namespace SurgSim
