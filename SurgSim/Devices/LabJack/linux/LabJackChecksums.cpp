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

#include "SurgSim/Devices/LabJack/linux/LabJackChecksums.h"

namespace SurgSim
{
namespace Devices
{
namespace LabJack
{

unsigned char normalChecksum8(const std::array<unsigned char, MAXIMUM_BUFFER>& bytes, int count)
{
	uint16_t accumulator = 0;

	//Sums bytes 1 to n-1 unsigned to a 2 byte value. Sums quotient and
	//remainder of 256 division.  Again, sums quotient and remainder of
	//256 division.
	for (int i = 1; i < count; ++i)
	{
		accumulator += static_cast<uint16_t>(bytes.at(i));
	}

	uint16_t quotient = accumulator / 256;
	accumulator = (accumulator - 256 * quotient) + quotient;
	quotient = accumulator / 256;

	return static_cast<unsigned char>((accumulator - 256 * quotient) + quotient);
}

uint16_t extendedChecksum16(const std::array<unsigned char, MAXIMUM_BUFFER>& bytes, int count)
{
	uint16_t accumulator = 0;

	//Sums bytes 6 to n-1 to an unsigned 2 byte value
	for (int i = 6; i < count; ++i)
	{
		accumulator += static_cast<uint16_t>(bytes.at(i));
	}

	return accumulator;
}

unsigned char extendedChecksum8(const std::array<unsigned char, MAXIMUM_BUFFER>& bytes)
{
	return normalChecksum8(bytes, 6);
}

void normalChecksum(std::array<unsigned char, MAXIMUM_BUFFER>* bytes, int count)
{
	(*bytes)[0] = normalChecksum8(*bytes, count);
}

void extendedChecksum(std::array<unsigned char, MAXIMUM_BUFFER>* bytes, int count)
{
	uint16_t accumulator = extendedChecksum16(*bytes, count);
	(*bytes)[4] = static_cast<unsigned char>(accumulator & 0xff);
	(*bytes)[5] = static_cast<unsigned char>((accumulator / 256) & 0xff);
	(*bytes)[0] = extendedChecksum8(*bytes);
}

};  // namespace LabJack
};  // namespace Devices
};  // namespace SurgSim
