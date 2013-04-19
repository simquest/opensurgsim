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

#include "SurgSim/Framework/AssertMessage.h"

#if defined(_WIN32)
#include <windows.h>
#endif // defined(_WIN32)


namespace SurgSim
{
namespace Framework
{


void AssertMessage::setFailureCallback(AssertMessage::DeathCallback callback)
{
	m_killMeNow = callback;
}

AssertMessage::DeathCallback AssertMessage::getFailureCallback()
{
	return m_killMeNow;
}

void AssertMessage::throwException(const std::string& errorMessage)
{
	throw AssertionFailure(errorMessage);
}

void AssertMessage::crashToDebugger(const std::string& errorMessage)
{
#if defined(_WIN32)
	DebugBreak();
#else  // not defined(_WIN32)
	*static_cast<volatile int*>(nullptr) = 123;
#endif // not defined(_WIN32)
}

AssertMessage::DeathCallback AssertMessage::m_killMeNow = AssertMessage::throwException;


};  // namespace Framework
};  // namespace SurgSim
