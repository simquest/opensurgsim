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

#include "SurgSim/Devices/LabJack/linux/LabJackTypeConverters.h"
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Devices
{
namespace LabJack
{

double doubleFromChars(const std::array<unsigned char, MAXIMUM_BUFFER>& bytes, int startIndex)
{
	uint32_t decimal = static_cast<uint32_t>(bytes.at(startIndex)) |
		(static_cast<uint32_t>(bytes.at(startIndex + 1)) << 8) |
		(static_cast<uint32_t>(bytes.at(startIndex + 2)) << 16) |
		(static_cast<uint32_t>(bytes.at(startIndex + 3)) << 24);

	uint32_t whole = static_cast<uint32_t>(bytes[startIndex + 4]) |
		(static_cast<uint32_t>(bytes.at(startIndex + 5)) << 8) |
		(static_cast<uint32_t>(bytes.at(startIndex + 6)) << 16) |
		(static_cast<uint32_t>(bytes.at(startIndex + 7)) << 24);

	return static_cast<double>(static_cast<int>(whole)) + static_cast<double>(decimal)/4294967296.0;
}

uint32_t uint32FromChars(const std::array<unsigned char, LabJack::MAXIMUM_BUFFER> &bytes, int startIndex, int count)
{
	SURGSIM_ASSERT(count <= 4) << __FUNCTION__ << " got a count of " << count << "; that is too large.";

	uint32_t value = 0;
	for (int i = count - 1; i > 0; --i)
	{
		value += bytes.at(startIndex + i);
		value = value << 8;
	}
	value += bytes.at(startIndex);
	return value;
}

uint16_t uint16FromChars(const std::array<unsigned char, LabJack::MAXIMUM_BUFFER> &bytes, int startIndex, int count)
{
	SURGSIM_ASSERT(count <= 2) << __FUNCTION__ << " got a count of " << count << "; that is too large.";

	uint16_t value = 0;
	if (count > 1)
	{
		value += bytes.at(startIndex + 1);
		value = value << 8;
	}
	value += bytes.at(startIndex);
	return value;
}

};  // namespace LabJack
};  // namespace Devices
};  // namespace SurgSim