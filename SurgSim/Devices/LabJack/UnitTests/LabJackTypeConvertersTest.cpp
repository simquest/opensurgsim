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
/// Tests for the LabJack specific type converters.

#include <gtest/gtest.h>

#include "SurgSim/Devices/LabJack/linux/LabJackConstants.h"
#include "SurgSim/Devices/LabJack/linux/LabJackTypeConverters.h"

namespace
{
	const double EPSILON = 1e-9;
}

TEST(LabJackTypeConvertersTest, DoubleFromChars)
{
	std::array<unsigned char, SurgSim::Devices::LabJack::MAXIMUM_BUFFER> bytes;
	for (int i = 0; i < 24; ++i)
	{
		bytes[i] = 0;
	}

	EXPECT_NEAR(0.0, SurgSim::Devices::LabJack::doubleFromChars(bytes, 0), EPSILON);

	bytes[12] = 2;
	EXPECT_NEAR(2.0, SurgSim::Devices::LabJack::doubleFromChars(bytes, 8), EPSILON);

	bytes[20] = 255;
	bytes[21] = 255;
	bytes[22] = 255;
	bytes[23] = 255;
	EXPECT_NEAR(-1.0, SurgSim::Devices::LabJack::doubleFromChars(bytes, 16), EPSILON);
}

TEST(LabJackTypeConvertersTest, Uint32FromChars)
{
	std::array<unsigned char, SurgSim::Devices::LabJack::MAXIMUM_BUFFER> bytes;
	for (int i = 0; i < 8; ++i)
	{
		bytes[i] = 0;
	}
	bytes[4] = 1;

	EXPECT_ANY_THROW(SurgSim::Devices::LabJack::uint32FromChars(bytes, 0, 5));
	EXPECT_EQ(0, SurgSim::Devices::LabJack::uint32FromChars(bytes, 0, 4));
	EXPECT_EQ(1 << 24, SurgSim::Devices::LabJack::uint32FromChars(bytes, 1, 4));
	EXPECT_EQ(1 << 16, SurgSim::Devices::LabJack::uint32FromChars(bytes, 2, 3));
	EXPECT_EQ(1, SurgSim::Devices::LabJack::uint32FromChars(bytes, 4, 4));
}

TEST(LabJackTypeConvertersTest, Uint16FromChars)
{
	std::array<unsigned char, SurgSim::Devices::LabJack::MAXIMUM_BUFFER> bytes;
	for (int i = 0; i < 4; ++i)
	{
		bytes[i] = 0;
	}
	bytes[2] = 1;

	EXPECT_ANY_THROW(SurgSim::Devices::LabJack::uint16FromChars(bytes, 0, 3));
	EXPECT_EQ(0, SurgSim::Devices::LabJack::uint16FromChars(bytes, 0, 2));
	EXPECT_EQ(1 << 8, SurgSim::Devices::LabJack::uint16FromChars(bytes, 1, 2));
	EXPECT_EQ(1, SurgSim::Devices::LabJack::uint16FromChars(bytes, 2, 1));
	EXPECT_EQ(1, SurgSim::Devices::LabJack::uint16FromChars(bytes, 2, 2));
}
