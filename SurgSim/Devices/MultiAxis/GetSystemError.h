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

#ifndef SURGSIM_DEVICES_MULTIAXIS_GETSYSTEMERROR_H
#define SURGSIM_DEVICES_MULTIAXIS_GETSYSTEMERROR_H

#include <stdint.h>
#include <string>

namespace SurgSim
{
namespace Devices
{
namespace Internal
{
/// Gets the most recent error code from the operating system.
/// The error code will correspond to the status of the most recent failed (or, in some cases, successful) call
/// to the operating system APIs from this thread.
/// \return	The OS-dependent system error code.
int64_t getSystemErrorCode();

/// Gets the system error text corresponding to the specified error code, or the most recent error text.
/// \param	errorCode	(Optional) The error code.  If omitted, the most recent OS error code will be used.
/// \return	The error text corresponding to the error code.
std::string getSystemErrorText(int64_t errorCode = getSystemErrorCode());

};  // namespace Internal
};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_GETSYSTEMERROR_H
