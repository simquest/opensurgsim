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

#include "SurgSim/Devices/MultiAxis/Win32HidDeviceHandle.h"

#include <stdint.h>

#include "SurgSim/Devices/MultiAxis/FileHandle.h"
#include "SurgSim/Devices/MultiAxis/GetSystemError.h"
#include <SurgSim/Framework/Log.h>

using SurgSim::Device::Internal::getSystemErrorCode;
using SurgSim::Device::Internal::getSystemErrorText;


namespace SurgSim
{
namespace Device
{

struct Win32HidDeviceHandle::State
{
public:
	explicit State(std::shared_ptr<SurgSim::Framework::Logger>&& logger_) :
		logger(std::move(logger_)),
		handle()
	{
	}

	/// The logger to use.
	std::shared_ptr<SurgSim::Framework::Logger> logger;
	/// The underlying device file handle.
	FileHandle handle;
};

Win32HidDeviceHandle::Win32HidDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger) :
	m_state(new Win32HidDeviceHandle::State(std::move(logger)))
{
}

Win32HidDeviceHandle::~Win32HidDeviceHandle()
{
}

std::unique_ptr<Win32HidDeviceHandle> Win32HidDeviceHandle::open(
	const std::string& path, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	std::unique_ptr<Win32HidDeviceHandle> object(new Win32HidDeviceHandle(std::move(logger)));
	if (! object->m_state->handle.openForReadingAndMaybeWriting(path))
	{
		object.reset();  // could not open the device handle; destroy the object again
	}
	return object;
}

bool Win32HidDeviceHandle::canRead() const
{
	return m_state->handle.canRead();
}

bool Win32HidDeviceHandle::canWrite() const
{
	return m_state->handle.canWrite();
}

bool Win32HidDeviceHandle::hasDataToRead() const
{
	return m_state->handle.hasDataToRead();
}

bool Win32HidDeviceHandle::readBytes(void* dataBuffer, size_t bytesToRead, size_t* bytesActuallyRead)
{
	return m_state->handle.readBytes(dataBuffer, bytesToRead, bytesActuallyRead);
}

// XXX HORRIBLE HACK!!!
void* Win32HidDeviceHandle::get() const
{
	return m_state->handle.get();
}

static inline int16_t signedShortData(unsigned char byte0, unsigned char byte1)
{
	return static_cast<int16_t>(static_cast<uint16_t>(byte0) | (static_cast<uint16_t>(byte1) << 8));
}

bool Win32HidDeviceHandle::updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated)
{
	// We can't keep reading while data is available, because we don't know how to tell when data is available.
	// Both WaitForSingleObject() and WaitForMultipleObjects() always claim data is available for 3DConnexion device
	// file handles.  So we just do it once, blocking until we have data.
	//
	// We also can't unblock the read once we initiate it (closing the handle has no effect).

	{
		unsigned char deviceBuffer[7*128];
		size_t numRead;
		if (! m_state->handle.readBytes(&deviceBuffer, sizeof(deviceBuffer), &numRead))
		{
			int64_t error = getSystemErrorCode();
			SURGSIM_LOG_WARNING(m_state->logger) << "RawMultiAxis: read failed with error " << error << ", " <<
				getSystemErrorText(error);
		}
		else if ((numRead >= 7) && (deviceBuffer[0] == 0x01))       // Translation
		{
			(*axisStates)[0] = signedShortData(deviceBuffer[1],  deviceBuffer[2]);
			(*axisStates)[1] = signedShortData(deviceBuffer[3],  deviceBuffer[4]);
			(*axisStates)[2] = signedShortData(deviceBuffer[5],  deviceBuffer[6]);
			*updated = true;

			if ((numRead >= 14) && (deviceBuffer[7] == 0x02))  // translation data may have rotation appended to it
			{
				(*axisStates)[3] = signedShortData(deviceBuffer[8],  deviceBuffer[9]);
				(*axisStates)[4] = signedShortData(deviceBuffer[10],  deviceBuffer[11]);
				(*axisStates)[5] = signedShortData(deviceBuffer[12],  deviceBuffer[13]);
			}
		}
		else if ((numRead >= 7) && (deviceBuffer[0] == 0x02))  // Rotation
		{
			(*axisStates)[3] = signedShortData(deviceBuffer[1],  deviceBuffer[2]);
			(*axisStates)[4] = signedShortData(deviceBuffer[3],  deviceBuffer[4]);
			(*axisStates)[5] = signedShortData(deviceBuffer[5],  deviceBuffer[6]);
			*updated = true;

			if ((numRead >= 14) && (deviceBuffer[7] == 0x01))  // rotation data may have translation appended to it
			{
				(*axisStates)[0] = signedShortData(deviceBuffer[8],  deviceBuffer[9]);
				(*axisStates)[1] = signedShortData(deviceBuffer[10],  deviceBuffer[11]);
				(*axisStates)[2] = signedShortData(deviceBuffer[12],  deviceBuffer[13]);
			}
		}
		else if ((numRead >= 2) && (deviceBuffer[0] == 0x03))  // Buttons
		{
			size_t currentByte = 1;  // Byte 0 specifies the packet type; data starts at byte 1
			unsigned char currentBit = 0x01;
			for (size_t i = 0;  i < (*buttonStates).size();  ++i)
			{
				(*buttonStates)[i] = ((deviceBuffer[currentByte] & currentBit) != 0);
				if (currentBit < 0x80)
				{
					currentBit = currentBit << 1;
				}
				else
				{
					currentBit = 0x01;
					++currentByte;
					if (currentByte >= numRead)  // out of data?
					{
						break;
					}
				}
			}
			*updated = true;
		}
	}

	return true;
}

};  // namespace Device
};  // namespace SurgSim
