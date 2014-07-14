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

#include <gtest/gtest.h>

#include <string>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/Runtime.h"

class MockAsset: public SurgSim::Framework::Asset
{
public:
	MockAsset() {}
	~MockAsset() {}

	virtual bool doInitialize(const std::string&) {return true;}
};

namespace SurgSim
{
namespace Framework
{

class AssetTest : public ::testing::Test
{
public:
	bool initialize(SurgSim::Framework::Asset& asset, const SurgSim::Framework::ApplicationData& appData)
	{
		return asset.initialize(appData);
	}
};



TEST_F(AssetTest, InitTest)
{
	EXPECT_NO_THROW(MockAsset t);

	MockAsset test;
	EXPECT_EQ("", test.getFileName());
	EXPECT_FALSE(test.isInitialized());
}

TEST_F(AssetTest, FileNameTest)
{
	MockAsset test;
	SurgSim::Framework::Runtime runtime("config.txt");
	std::string fileName = "TestFileName";

	EXPECT_NO_THROW(test.setFileName(fileName));
	EXPECT_EQ(fileName, test.getFileName());
}

TEST_F(AssetTest, InitializationTest)
{
	SurgSim::Framework::Runtime runtime("config.txt");
	{
		MockAsset test;
		auto applicationData = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");

		// Call 'initialize()' without setting file name will fail.
		EXPECT_FALSE(initialize(test, *applicationData));
		EXPECT_FALSE(test.isInitialized());
	}

	{
		MockAsset test;

		auto applicationData = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");

		// Loading non-exist file will fail.
		test.setFileName("Non-exist-file");
		EXPECT_FALSE(test.isInitialized());

		// Asset::initialize() is implicitly called by Asset::setFileName()
		// A second call to Asset::initialize() will throw.
		EXPECT_ANY_THROW(initialize(test, *applicationData));

		// Since setFileName() now calls Asset::initialize() internally which asserts on double calls,
		// this makes setFileName() can be called only once.
		EXPECT_ANY_THROW(test.setFileName("AssetTestData/DummyFile.txt"));
		// 'initialize()' can only be called once (no matter what the result the first time was).
		EXPECT_ANY_THROW(initialize(test, *applicationData));
		EXPECT_FALSE(test.isInitialized());
	}

	{
		MockAsset test;
		auto applicationData = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");

		test.setFileName("AssetTestData/DummyFile.txt");
		// Asset::initialize() is implicitly called by Asset::setFileName()
		// A second call will throw.
		EXPECT_ANY_THROW(initialize(test, *applicationData));
		EXPECT_TRUE(test.isInitialized());

		// 'initialize()' can only be called once (no matter what the result the first time was).
		EXPECT_ANY_THROW(initialize(test, *applicationData));
		// However, 'isInitialzed()' won't be affected.
		EXPECT_TRUE(test.isInitialized());
	}
}

}; // Framework
}; // SurgSim