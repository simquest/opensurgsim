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

#ifndef SURGSIM_DEVICES_MULTIAXIS_FILEDESCRIPTOR_H
#define SURGSIM_DEVICES_MULTIAXIS_FILEDESCRIPTOR_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string>

#include <SurgSim/Framework/Assert.h>


namespace SurgSim
{
namespace Device
{

/// A wrapper for an UNIX-style integer file descriptor.
/// Allows callers to implement RAII-style resource management.
class FileDescriptor
{
public:
	FileDescriptor() :
		m_descriptor(INVALID_VALUE),
		m_canRead(false),
		m_canWrite(false)
	{
	}

	FileDescriptor(FileDescriptor&& other) :
		m_descriptor(other.m_descriptor),
		m_canRead(other.m_canRead),
		m_canWrite(other.m_canWrite)
	{
		other.m_descriptor = INVALID_VALUE;  // take ownership
	}

	FileDescriptor& operator=(FileDescriptor&& other)
	{
		m_descriptor = other.m_descriptor;
		m_canRead = other.m_canRead;
		m_canWrite = other.m_canWrite;
		other.m_descriptor = INVALID_VALUE;  // take ownership
		return *this;
	}

	~FileDescriptor()
	{
		reset();
	}

	bool isValid() const
	{
		return (m_descriptor != INVALID_VALUE);
	}

	bool canRead() const
	{
		return isValid() && m_canRead;
	}

	bool canWrite() const
	{
		return isValid() && m_canWrite;
	}

	int get() const
	{
		SURGSIM_ASSERT(m_descriptor != INVALID_VALUE);
		return m_descriptor;
	}

	bool openForReadingAndWriting(const std::string& path)
	{
		reset();
		m_descriptor = open(path.c_str(), O_RDWR);
		m_canRead = true;
		m_canWrite = true;
		return isValid();
	}

	bool openForReading(const std::string& path)
	{
		reset();
		m_descriptor = open(path.c_str(), O_RDONLY);
		m_canRead = true;
		m_canWrite = false;
		return isValid();
	}

	bool openForWriting(const std::string& path)
	{
		reset();
		m_descriptor = open(path.c_str(), O_WRONLY);
		m_canRead = false;
		m_canWrite = true;
		return isValid();
	}

	bool openForReadingAndMaybeWriting(const std::string& path)
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

	void reset()
	{
		if (m_descriptor != INVALID_VALUE)
		{
			close(m_descriptor);
			m_descriptor = INVALID_VALUE;
		}
	}

private:
	FileDescriptor(const FileDescriptor& other) = delete;
	FileDescriptor& operator=(const FileDescriptor& other) = delete;

	static const int INVALID_VALUE = -1;

	int m_descriptor;
	bool m_canRead;
	bool m_canWrite;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_FILEDESCRIPTOR_H
