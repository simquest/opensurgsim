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

#include "SurgSim/Devices/MultiAxis/GetSystemError.h"

#include <errno.h>
#include <string.h>


namespace SurgSim
{
namespace Devices
{
namespace Internal
{

int64_t getSystemErrorCode()
{
	return errno;
}


// Helps retrieve the output location for the XSI version of strerror_r.
static inline const char* systemErrorTextHelper(const char* buffer, int returnValue)
{
	if (returnValue != 0)
	{
		return nullptr;
	}
	return buffer;
}

// Helps retrieve the output location for the GNU version of strerror_r.
static inline const char* systemErrorTextHelper(const char* buffer, const char* returnValue)
{
	return returnValue;
}

std::string getSystemErrorText(int64_t errorCode)
{
	const size_t BUFFER_SIZE = 1024;
	char errorBuffer[BUFFER_SIZE];
	errorBuffer[0] = '\0';

	// Unfortunately, on Linux you can't really know if you will get the XSI version or the GNU version of
	// strerror_r.  Fortunately, the arguments are the same (only return type differs), so we can cheat.
	const char* message = systemErrorTextHelper(errorBuffer, strerror_r(errorCode, errorBuffer, sizeof(errorBuffer)));
	if (message == nullptr || message[0] == '\0')
	{
		snprintf(errorBuffer, sizeof(errorBuffer), "<system error %d>", static_cast<int>(errorCode));
		message = errorBuffer;
	}
	return std::string(message);
}

};  // namespace Internal
};  // namespace Devices
};  // namespace SurgSim
