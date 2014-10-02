// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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
/// Tests for the TokenStream class.

#include "gtest/gtest.h"

#include "SurgSim/DataStructures/TokenStream.h"

using SurgSim::DataStructures::TokenStream;

TEST(TokenStream, ParseInt)
{
	std::stringstream ss;
	ss << "10 12";
	EXPECT_TRUE(ss.good());

	TokenStream tokenStream(&ss);

	int parsedInt;
	EXPECT_TRUE(tokenStream.parse(&parsedInt));
	EXPECT_EQ(10, parsedInt);

	EXPECT_TRUE(tokenStream.parse(&parsedInt));
	EXPECT_EQ(12, parsedInt);

	EXPECT_FALSE(tokenStream.parse(&parsedInt));
}

TEST(TokenStream, ParseDouble)
{
	std::stringstream ss;
	ss << "10.0 3.14";
	TokenStream tokenStream(&ss);

	double parsedDouble;
	EXPECT_TRUE(tokenStream.parse(&parsedDouble));
	EXPECT_EQ(10.0, parsedDouble);

	EXPECT_TRUE(tokenStream.parse(&parsedDouble));
	EXPECT_EQ(3.14, parsedDouble);

	EXPECT_FALSE(tokenStream.parse(&parsedDouble));
}

TEST(TokenStream, ParseString)
{
	std::stringstream ss;
	ss << "hello world";
	TokenStream tokenStream(&ss);

	std::string parsedString;
	EXPECT_TRUE(tokenStream.parse(&parsedString));
	EXPECT_EQ("hello", parsedString);

	EXPECT_TRUE(tokenStream.parse(&parsedString));
	EXPECT_EQ("world", parsedString);

	EXPECT_FALSE(tokenStream.parse(&parsedString));
}

TEST(TokenStream, ParseVector)
{
	std::stringstream ss;
	ss << "1.0 2.0 3.0 4.0 5.0 6.0";
	TokenStream tokenStream(&ss);

	SurgSim::Math::Vector3d parsedVector;
	EXPECT_TRUE(tokenStream.parse(&parsedVector));
	EXPECT_TRUE(SurgSim::Math::Vector3d(1, 2, 3).isApprox(parsedVector));

	EXPECT_TRUE(tokenStream.parse(&parsedVector));
	EXPECT_TRUE(SurgSim::Math::Vector3d(4, 5, 6).isApprox(parsedVector));

	EXPECT_FALSE(tokenStream.parse(&parsedVector));
}

TEST(TokenStream, ParseQuaternion)
{
	std::stringstream ss;
	ss << "1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0";
	TokenStream tokenStream(&ss);

	SurgSim::Math::Quaterniond parsedQuat;
	EXPECT_TRUE(tokenStream.parse(&parsedQuat));
	EXPECT_TRUE(SurgSim::Math::Quaterniond(4, 1, 2, 3).isApprox(parsedQuat));

	EXPECT_TRUE(tokenStream.parse(&parsedQuat));
	EXPECT_TRUE(SurgSim::Math::Quaterniond(8, 5, 6, 7).isApprox(parsedQuat));

	EXPECT_FALSE(tokenStream.parse(&parsedQuat));
}