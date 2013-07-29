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

#include <string>


namespace SurgSim
{
namespace Device
{

/// A wrapper for an UNIX-style integer file descriptor.
/// Allows callers to implement RAII-style resource management.
class FileDescriptor
{
public:
	FileDescriptor();

	FileDescriptor(FileDescriptor&& other);

	FileDescriptor& operator=(FileDescriptor&& other);

	~FileDescriptor();

	bool isValid() const;

	bool canRead() const;

	bool canWrite() const;

	int get() const;

	bool openForReadingAndWriting(const std::string& path);

	bool openForReading(const std::string& path);

	bool openForWriting(const std::string& path);

	bool openForReadingAndMaybeWriting(const std::string& path);

	void reset();

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
