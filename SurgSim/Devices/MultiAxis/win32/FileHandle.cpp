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

#include "SurgSim/Devices/MultiAxis/win32/FileHandle.h"

#undef  _WIN32_WINNT
#define _WIN32_WINNT 0x0501   // request Windows XP-compatible SDK APIs
#undef  WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // do not automatically include WinSock 1 and some other header files
#include <windows.h>

#include <string>

#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Devices
{

// TODO(advornik): This would be a perfect use for "constexpr", but VS 2012 doesn't support it...
static const FileHandle::RawHandleType INVALID_VALUE = INVALID_HANDLE_VALUE;


FileHandle::FileHandle() :
	m_handle(INVALID_VALUE),
	m_canRead(false),
	m_canWrite(false),
	m_openFlags(0)
{
}

FileHandle::FileHandle(FileHandle&& other) :
	m_handle(other.m_handle),
	m_canRead(other.m_canRead),
	m_canWrite(other.m_canWrite),
	m_openFlags(other.m_openFlags)
{
	other.m_handle = INVALID_VALUE;  // take ownership
}

FileHandle& FileHandle::operator=(FileHandle&& other)
{
	m_handle = other.m_handle;
	m_canRead = other.m_canRead;
	m_canWrite = other.m_canWrite;
	m_openFlags = other.m_openFlags;
	other.m_handle = INVALID_VALUE;  // take ownership
	return *this;
}

FileHandle::~FileHandle()
{
	reset();
}

bool FileHandle::isValid() const
{
	return (m_handle && (m_handle != INVALID_VALUE));
}

bool FileHandle::canRead() const
{
	return isValid() && m_canRead;
}

bool FileHandle::canWrite() const
{
	return isValid() && m_canWrite;
}

FileHandle::RawHandleType FileHandle::get() const
{
	SURGSIM_ASSERT(isValid());
	return m_handle;
}

bool FileHandle::openForReadingAndWriting(const std::string& path)
{
	reset();
	m_handle = CreateFile(path.c_str(),
		GENERIC_READ | GENERIC_WRITE | SYNCHRONIZE,   FILE_SHARE_READ | FILE_SHARE_WRITE,   NULL,
		OPEN_EXISTING,   FILE_ATTRIBUTE_NORMAL | static_cast<DWORD>(m_openFlags),   NULL);
	m_canRead = true;
	m_canWrite = true;
	return isValid();
}

bool FileHandle::openForReading(const std::string& path)
{
	reset();
	m_handle = CreateFile(path.c_str(),
		GENERIC_READ | SYNCHRONIZE,   FILE_SHARE_READ | FILE_SHARE_WRITE,   NULL,
		OPEN_EXISTING,   FILE_ATTRIBUTE_NORMAL | static_cast<DWORD>(m_openFlags),   NULL);
	m_canRead = true;
	m_canWrite = false;
	return isValid();
}

bool FileHandle::openForWriting(const std::string& path)
{
	reset();
	m_handle = CreateFile(path.c_str(),
		GENERIC_WRITE | SYNCHRONIZE,   FILE_SHARE_READ | FILE_SHARE_WRITE,   NULL,
		OPEN_EXISTING,   FILE_ATTRIBUTE_NORMAL | static_cast<DWORD>(m_openFlags),   NULL);
	m_canRead = false;
	m_canWrite = true;
	return isValid();
}

bool FileHandle::openForReadingAndMaybeWriting(const std::string& path)
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

void FileHandle::reset()
{
	if (m_handle != INVALID_VALUE)
	{
		CloseHandle(m_handle);
		m_handle = INVALID_VALUE;
	}
}

void FileHandle::setFileOpenFlags(uint64_t flags)
{
	SURGSIM_ASSERT(! isValid()) << "Flags need to be set before the file is opened!";
	m_openFlags = flags;
}

uint64_t FileHandle::getFileOpenFlags() const
{
	return m_openFlags;
}

bool FileHandle::hasDataToRead() const
{
	if (! canRead())
	{
		return false;
	}

	//const DWORD timeoutMsec = 10;
	const DWORD nonBlockingOnly = 0;
	DWORD status = WaitForSingleObject(m_handle, nonBlockingOnly);
	return (status == WAIT_OBJECT_0);
}

bool FileHandle::readBytes(void* dataBuffer, unsigned int bytesToRead, unsigned int* bytesActuallyRead)
{
	SURGSIM_ASSERT(canRead());
	DWORD numBytesRead = 0;
	if (ReadFile(m_handle, dataBuffer, bytesToRead, &numBytesRead, NULL) != TRUE)
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