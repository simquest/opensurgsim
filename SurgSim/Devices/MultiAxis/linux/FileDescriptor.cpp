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

#include "SurgSim/Devices/MultiAxis/linux/FileDescriptor.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

#include <string>

#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Devices
{

FileDescriptor::FileDescriptor() :
	m_descriptor(INVALID_VALUE),
	m_canRead(false),
	m_canWrite(false)
{
}

FileDescriptor::FileDescriptor(FileDescriptor&& other) :
	m_descriptor(other.m_descriptor),
	m_canRead(other.m_canRead),
	m_canWrite(other.m_canWrite)
{
	other.m_descriptor = INVALID_VALUE;  // take ownership
}

FileDescriptor& FileDescriptor::operator=(FileDescriptor&& other)
{
	m_descriptor = other.m_descriptor;
	m_canRead = other.m_canRead;
	m_canWrite = other.m_canWrite;
	other.m_descriptor = INVALID_VALUE;  // take ownership
	return *this;
}

FileDescriptor::~FileDescriptor()
{
	reset();
}

bool FileDescriptor::isValid() const
{
	return (m_descriptor != INVALID_VALUE);
}

bool FileDescriptor::canRead() const
{
	return isValid() && m_canRead;
}

bool FileDescriptor::canWrite() const
{
	return isValid() && m_canWrite;
}

int FileDescriptor::get() const
{
	SURGSIM_ASSERT(m_descriptor != INVALID_VALUE);
	return m_descriptor;
}

bool FileDescriptor::openForReadingAndWriting(const std::string& path)
{
	reset();
	m_descriptor = open(path.c_str(), O_RDWR);
	m_canRead = true;
	m_canWrite = true;
	return isValid();
}

bool FileDescriptor::openForReading(const std::string& path)
{
	reset();
	m_descriptor = open(path.c_str(), O_RDONLY);
	m_canRead = true;
	m_canWrite = false;
	return isValid();
}

bool FileDescriptor::openForWriting(const std::string& path)
{
	reset();
	m_descriptor = open(path.c_str(), O_WRONLY);
	m_canRead = false;
	m_canWrite = true;
	return isValid();
}

bool FileDescriptor::openForReadingAndMaybeWriting(const std::string& path)
{
	if (! openForReadingAndWriting(path))
	{
		if (! openForReading(path))
		{
			return false;
		}
	}
	return true;
}

void FileDescriptor::reset()
{
	if (m_descriptor != INVALID_VALUE)
	{
		close(m_descriptor);
		m_descriptor = INVALID_VALUE;
	}
}

bool FileDescriptor::hasDataToRead() const
{
	if (! canRead())
	{
		return false;
	}

	struct pollfd pollData[1];
	pollData[0].fd = m_descriptor;
	pollData[0].events = POLLIN;

	//const int timeoutMsec = 10;
	const int nonBlockingOnly = 0;
	int status = poll(pollData, 1, nonBlockingOnly);
	return (status > 0);
}

bool FileDescriptor::readBytes(void* dataBuffer, size_t bytesToRead, size_t* bytesActuallyRead)
{
	SURGSIM_ASSERT(canRead());
	ssize_t numBytesRead = read(m_descriptor, dataBuffer, bytesToRead);
	if (numBytesRead < 0)
	{
		*bytesActuallyRead = 0;
		return false;
	}
	else
	{
		*bytesActuallyRead = numBytesRead;
		return true;
	}
}

};  // namespace Devices
};  // namespace SurgSim
