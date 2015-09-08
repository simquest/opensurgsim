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

#ifndef SURGSIM_DEVICES_MULTIAXIS_LINUX_FILEDESCRIPTOR_H
#define SURGSIM_DEVICES_MULTIAXIS_LINUX_FILEDESCRIPTOR_H

#include <string>


namespace SurgSim
{
namespace Devices
{

/// A wrapper for an UNIX-style integer file descriptor.
/// Allows callers to implement RAII-style resource management.
class FileDescriptor
{
public:
	/// Default constructor.
	/// Initializes the file descriptor to an invalid state.
	FileDescriptor();

	/// Move constructor.
	/// \param [in,out]	other	The object to move.  The original object will be invalidated.
	FileDescriptor(FileDescriptor&& other);

	/// Move assignment operator.
	/// \param [in,out]	other	The object to move.  The original object will be invalidated.
	/// \return	A reference to this object.
	FileDescriptor& operator=(FileDescriptor&& other);

	/// Destructor.
	~FileDescriptor();

	/// Checks if the file descriptor is valid, i.e. has been opened.
	/// \return	true if valid, false if not.
	bool isValid() const;

	/// Determines if the file descriptor can be read from.
	/// \return	true if the descriptor has been open for reading.
	bool canRead() const;

	/// Determines if the file descriptor can be written to.
	/// \return	true if the descriptor has been open for writing.
	bool canWrite() const;

	/// Checks whether this object has data available to be read.
	/// \return	true if there is data currently available.
	bool hasDataToRead() const;

	/// Reads bytes from the file descriptor.
	/// \param [out]	dataBuffer	Buffer to read into.  Must have room for at least bytesToRead bytes of data.
	/// \param	bytesToRead	The number of bytes to try reading.  Actual number of bytes received may be smaller.
	/// \param [out]	bytesActuallyRead	The number of bytes that were actually read into the buffer.
	/// \return	true if it succeeds, false if it fails.
	bool readBytes(void* dataBuffer, size_t bytesToRead, size_t* bytesActuallyRead);

	/// Gets the raw underlying OS file descriptor.
	/// \return	The raw file descriptor.
	int get() const;

	/// Attempts to open the file descriptor for reading and writing.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForReadingAndWriting(const std::string& path);

	/// Attempts to open the file descriptor for reading only.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForReading(const std::string& path);

	/// Attempts to open the file descriptor for writing only.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForWriting(const std::string& path);

	/// Attempts to open the file descriptor for reading and (if permissions allow it) writing.
	/// \param	path	Full pathname of the file.
	/// \return	true if it succeeds, false if it fails.
	bool openForReadingAndMaybeWriting(const std::string& path);

	/// Resets the file descriptor back to an invalid state.
	/// If the descriptor was open, it will be closed.
	void reset();

private:
	FileDescriptor(const FileDescriptor& other) = delete;
	FileDescriptor& operator=(const FileDescriptor& other) = delete;

	static const int INVALID_VALUE = -1;

	int m_descriptor;
	bool m_canRead;
	bool m_canWrite;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_LINUX_FILEDESCRIPTOR_H
