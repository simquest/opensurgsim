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

#include <SurgSim/Devices/MultiAxis/FileDescriptor.h>


namespace SurgSim
{
namespace Device
{

struct LinuxInputDeviceHandle::State
{
public:
	State() :
		handle()
	{
	}

	FileDescriptor handle;
};

LinuxInputDeviceHandle::LinuxInputDeviceHandle() :
	m_state(new LinuxInputDeviceHandle::State)
{
}

LinuxInputDeviceHandle::~LinuxInputDeviceHandle()
{
}

std::unique_ptr<LinuxInputDeviceHandle> LinuxInputDeviceHandle::open(const std::string& path)
{
	std::unique_ptr<LinuxInputDeviceHandle> object(new LinuxInputDeviceHandle);
	if (! object->m_state->handle.openForReadingAndMaybeWriting(path))
	{
		object.reset();  // could not open the device handle; destroy the object again
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

};  // namespace Device
};  // namespace SurgSim
