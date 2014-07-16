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
/// Tests for SURGSIM_ASSERT() and SURGSIM_FAILURE().

#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/Runtime.h"

class MockAsset: public SurgSim::Framework::Asset
{
public:
	MockAsset() {}
	~MockAsset() {}

	virtual bool doLoad(const std::string& fileName)
	{
		bool result = false;
		std::ifstream in(fileName);

		if (in.is_open())
		{
			result = true;
		}
		return result;
	}
};

namespace SurgSim
{
namespace Framework
{

class AssetTest : public ::testing::Test
{
public:
	void SetUp()
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	}

	void setFileName(SurgSim::Framework::Asset& asset, const std::string& fileName)
	{
		asset.setFileName(fileName);
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
};

TEST_F(AssetTest, InitTest)
{
	EXPECT_NO_THROW(MockAsset t);

	MockAsset test;
	EXPECT_EQ("", test.getFileName());
	EXPECT_FALSE(test.load(""));
}

TEST_F(AssetTest, FileNameTest)
{
	MockAsset test;
	std::string fileName1 = "TestFileName1";
	std::string fileName2 = "TestFileName2";

	EXPECT_NO_THROW(EXPECT_FALSE(test.load(fileName1)));
	EXPECT_EQ(fileName1, test.getFileName());

	EXPECT_NO_THROW(setFileName(test, fileName2));
	EXPECT_EQ(fileName2, test.getFileName());
}

TEST_F(AssetTest, LoadTest)
{
	MockAsset test;

	// Call 'Asset::load()' with empty file name will fail.
	EXPECT_FALSE(test.load(""));
	EXPECT_EQ("", test.getFileName());

	// Loading non-exist file will fail, but the internal file name recorded by Asset will be updated.
	std::string invalidFileName("Non-exist-file");
	EXPECT_FALSE(test.load(invalidFileName));
	EXPECT_EQ(invalidFileName, test.getFileName());

	// Loading existing file should success and internal file name recorded by Asset will be updated.
	std::string validDummyFile("AssetTestData/DummyFile.txt");
	EXPECT_TRUE(test.load(validDummyFile));
	EXPECT_EQ(validDummyFile, test.getFileName());

	// Loading same existing file again should success and internal file name will be the same.
	EXPECT_TRUE(test.load(validDummyFile));
	EXPECT_EQ(validDummyFile, test.getFileName());
}

}; // Framework
}; // SurgSim