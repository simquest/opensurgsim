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

#undef  _WIN32_WINNT
#define _WIN32_WINNT 0x0501   // request Windows XP-compatible SDK APIs
#undef  WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // do not automatically include WinSock 1 and some other header files
#include <windows.h>


namespace SurgSim
{
namespace Devices
{
namespace Internal
{

int64_t getSystemErrorCode()
{
	return GetLastError();
}

std::string getSystemErrorText(int64_t errorCode)
{
	const size_t BUFFER_SIZE = 1024;
	char errorBuffer[BUFFER_SIZE];
	if (FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, 0, static_cast<DWORD>(errorCode), 0,
		errorBuffer, BUFFER_SIZE-1, nullptr) <= 0)
	{
		_snprintf(errorBuffer, BUFFER_SIZE, "<system error %u>", static_cast<unsigned int>(errorCode));
	}
	errorBuffer[BUFFER_SIZE-1] = '\0';

	// Strip terminal whitespace, if any.
	// Note that this approach only works for fixed-width characters, which is why we use the ASCII API above.
	const size_t end = strnlen(errorBuffer, BUFFER_SIZE-1);
	if ((end > 0) && isspace(errorBuffer[end-1]))
	{
		size_t lastWhitespace = end - 1;
		while ((lastWhitespace > 0) && isspace(errorBuffer[lastWhitespace-1]))
		{
			--lastWhitespace;
		}
		errorBuffer[lastWhitespace] = '\0';
	}

	return std::string(errorBuffer);
}

};  // namespace Internal
};  // namespace Devices
};  // namespace SurgSim