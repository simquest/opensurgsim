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

#include <SurgSim/Devices/MultiAxis/FileHandle.h>


namespace SurgSim
{
namespace Device
{

struct Win32HidDeviceHandle::State
{
public:
	State() :
		handle()
	{
	}

	FileHandle handle;
};

Win32HidDeviceHandle::Win32HidDeviceHandle() :
	m_state(new Win32HidDeviceHandle::State)
{
}

Win32HidDeviceHandle::~Win32HidDeviceHandle()
{
}

std::unique_ptr<Win32HidDeviceHandle> Win32HidDeviceHandle::open(const std::string& path)
{
	std::unique_ptr<Win32HidDeviceHandle> object(new Win32HidDeviceHandle);
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

};  // namespace Device
};  // namespace SurgSim
