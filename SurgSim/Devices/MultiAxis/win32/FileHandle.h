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

#ifndef SURGSIM_DEVICES_MULTIAXIS_WIN32_FILEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_WIN32_FILEHANDLE_H

#include <stdint.h>
#include <string>


namespace SurgSim
{
namespace Devices
{

/// A wrapper for an Windows-style HANDLE file descriptor.
/// Allows callers to implement RAII-style resource management.
class FileHandle
{
public:
	/// Type of the raw handle used by the operating system.
	/// Defined this way to avoid including <windows.h> just for the sake of defining HANDLE.
	typedef void* RawHandleType;

	/// Default constructor.
	/// Initializes the file handle to an invalid state.
	FileHandle();

	/// Move constructor.
	/// \param [in,out]	other	The object to move.  The original object will be invalidated.
	FileHandle(FileHandle&& other);

	/// Move assignment operator.
	/// \param [in,out]	other	The object to move.  The original object will be invalidated.
	/// \return	A reference to this object.
	FileHandle& operator=(FileHandle&& other);

	/// Destructor.
	~FileHandle();

	/// Checks if the file handle is valid, i.e. has been opened.
	/// \return	true if valid, false if not.
	bool isValid() const;

	/// Determines if the file handle can be read from.
	/// \return	true if the handle has been open for reading.
	bool canRead() const;

	/// Determines if the file handle can be written to.
	/// \return	true if the handle has been open for writing.
	bool canWrite() const;

	/// Checks whether this object has data available to be read.
	/// \return	true if there is data currently available.
	bool hasDataToRead() const;

	/// Reads bytes from the file handle.
	/// \param [out]	dataBuffer	Buffer to read into.  Must have room for at least bytesToRead bytes of data.
	/// \param	bytesToRead	The number of bytes to try reading.  Actual number of bytes received may be smaller.
	/// \param [out]	bytesActuallyRead	The number of bytes that were actually read into the buffer.
	/// \return	true if it succeeds, false if it fails.
	bool readBytes(void* dataBuffer, unsigned int bytesToRead, unsigned int* bytesActuallyRead);

	/// Gets the raw underlying OS file handle.
	/// \return	The raw file handle.
	RawHandleType get() const;

	/// Attempts to open the file handle for reading and writing.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForReadingAndWriting(const std::string& path);

	/// Attempts to open the file handle for reading only.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForReading(const std::string& path);

	/// Attempts to open the file handle for writing only.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForWriting(const std::string& path);

	/// Attempts to open the file handle for reading and (if permissions allow it) writing.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForReadingAndMaybeWriting(const std::string& path);

	/// Sets the flags that will be passed to CreateFile when opening the file.
	/// \param flags	The flags, a combination of zero or more Windows FILE_FLAG_* flags.
	void setFileOpenFlags(uint64_t flags);

	/// Gets the flags that will be passed to CreateFile when opening the file.
	/// \return	The value passed to setFileOpenFlags (or if never set, a default value).
	uint64_t getFileOpenFlags() const;

	/// Resets the file handle back to an invalid state.
	/// If the handle was open, it will be closed.
	void reset();

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	FileHandle(const FileHandle& other) /*= delete*/;
	FileHandle& operator=(const FileHandle& other) /*= delete*/;

	RawHandleType m_handle;
	bool m_canRead;
	bool m_canWrite;
	uint64_t m_openFlags;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_WIN32_FILEHANDLE_H
