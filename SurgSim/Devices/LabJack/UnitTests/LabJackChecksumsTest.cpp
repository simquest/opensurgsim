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

/// \file
/// Tests for the LabJack specific checksum functions.

#include <gtest/gtest.h>

#include "SurgSim/Devices/LabJack/linux/LabJackConstants.h"
#include "SurgSim/Devices/LabJack/linux/LabJackChecksums.h"

using SurgSim::Devices::LabJack::extendedChecksum;
using SurgSim::Devices::LabJack::extendedChecksum8;
using SurgSim::Devices::LabJack::extendedChecksum16;
using SurgSim::Devices::LabJack::normalChecksum;
using SurgSim::Devices::LabJack::normalChecksum8;

TEST(LabJackChecksumsTest, NormalChecksum)
{
	// Sum less than 256
	std::array<unsigned char, SurgSim::Devices::LabJack::MAXIMUM_BUFFER> bytes;
	unsigned char fillValue = 2;
	bytes.fill(fillValue);
	int count = 10;
	int expectedValue = fillValue * (count - 1);
	EXPECT_EQ(expectedValue, normalChecksum8(bytes, count));
	normalChecksum(&bytes, count);
	EXPECT_EQ(expectedValue, bytes[0]);

	// Sum greater than 256, quotient + remainder < 256
	fillValue = 100;
	bytes.fill(fillValue);
	count = 4;
	int sum = fillValue * (count - 1); // 300
	int quotient = sum / 256; // 1
	int remainder = sum % 256; // 44
	expectedValue = quotient + remainder;
	EXPECT_EQ(expectedValue, normalChecksum8(bytes, count));
	normalChecksum(&bytes, count);
	EXPECT_EQ(expectedValue, bytes[0]);

	// Sum greater than 256, quotient + remainder > 256
	fillValue = 255;
	bytes.fill(fillValue);
	bytes[4] = 2;
	count = 5;
	// sum = 767, quotient = 2, remainder = 255, quotient + remainder = 257
	// second_quotient = 1, second_remainder = 1, second_quotient + second_remainder = 2
	expectedValue = 2;
	EXPECT_EQ(expectedValue, normalChecksum8(bytes, count));
	normalChecksum(&bytes, count);
	EXPECT_EQ(expectedValue, bytes[0]);
}


TEST(LabJackChecksumsTest, ExtendedChecksum)
{
	// Sums less than 256
	std::array<unsigned char, SurgSim::Devices::LabJack::MAXIMUM_BUFFER> bytes;
	unsigned char fillValue = 2;
	bytes.fill(fillValue);
	int count = 10;
	int expectedValue16 = (count - 6) * fillValue; // 4 * 2 = 8
	EXPECT_EQ(expectedValue16, extendedChecksum16(bytes, count));
	int expectedValue8 = (6 - 1) * fillValue; // 5 * 2 = 10
	EXPECT_EQ(expectedValue8, extendedChecksum8(bytes));

	extendedChecksum(&bytes, count);
	EXPECT_EQ(expectedValue16, bytes[4]);
	EXPECT_EQ(0, bytes[5]);
	// extendedChecksum alters the buffer before setting bytes[0] to the return value of extendedChecksum8.
	EXPECT_EQ(expectedValue8 - 2 * fillValue + expectedValue16, bytes[0]);

	// Sum greater than 256, quotient + remainder < 256
	fillValue = 100;
	bytes.fill(fillValue);
	count = 20;
	expectedValue16 = (count - 6) * fillValue; // 14 * 100 = 1400
	EXPECT_EQ(expectedValue16, extendedChecksum16(bytes, count));
	expectedValue8 = 245; // sum = 5 * 100 = 500, quotient = 1, remainder = 244, quotient + remainder = 245
	EXPECT_EQ(expectedValue8, extendedChecksum8(bytes));

	extendedChecksum(&bytes, count);
	EXPECT_EQ(120, bytes[4]);
	EXPECT_EQ(5, bytes[5]);
	// extendedChecksum alters the buffer before setting bytes[0] to the return value of extendedChecksum8.
	// sum = 100 + 100 + 100 + 120 + 5 = 425, quotient = 1, remainder = 169, quotient + remainder = 170
	EXPECT_EQ(170, bytes[0]);
}
