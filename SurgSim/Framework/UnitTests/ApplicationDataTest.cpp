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

#include <gtest/gtest.h>
#include <vector>
#include <algorithm>

#include "SurgSim/Framework/ApplicationData.h"

#include <boost/filesystem.hpp>

using SurgSim::Framework::ApplicationData;
using boost::filesystem::path;

namespace std
{

::std::ostream& operator<<(::std::ostream& os, const vector<std::string>& content)
{
	os << "(";

	for (size_t i = 0; i < content.size(); ++i)
	{
		if (i > 0)
		{
			os << ", ";
		}
		os << "\"" << content[i] << "\"";
	}
	os << ")";
	return os;
}

}

::testing::AssertionResult isContained(const std::string& expected,
									   const std::vector<std::string>& argument)
{
	if (std::find(argument.cbegin(), argument.cend(), expected) != argument.cend())
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << "\"" << expected << "\"" << " not contained in " << argument;
	}
}

::testing::AssertionResult fileIsFoundCorrectly(
	const ApplicationData& data,
	const std::string& searchFileName,
	const std::string& expectedDirectoryName)
{
	// First file should be there and in directory1
	std::string fileName = data.findFile(searchFileName);
	boost::filesystem::path filePath(fileName);

	if (! filePath.is_absolute())
	{
		return ::testing::AssertionFailure() << "Result not absolute path <" << fileName <<
			   "> expected " << boost::filesystem::canonical(filePath);
	}
	if (! boost::filesystem::exists(filePath))
	{
		return ::testing::AssertionFailure() << "Result does not exist <" << fileName << ">";
	}
	if (fileName != filePath.make_preferred().string())
	{
		return ::testing::AssertionFailure() << "Result not system format path " << fileName <<
			   " expected " << filePath.make_preferred().string();
	}
	if (boost::filesystem::path(searchFileName).filename() != filePath.filename())
	{
		return ::testing::AssertionFailure() << "Expected " << searchFileName <<
			   " Result " << filePath.filename();
	}
	if (fileName.find(expectedDirectoryName) == std::string::npos)
	{
		return ::testing::AssertionFailure() << "Expected the file to be in subdirectory " <<
			   expectedDirectoryName << " but is not " << fileName;
	}
	return ::testing::AssertionSuccess();
}

TEST(ApplicationDataTest, InitTest)
{
	std::vector<std::string> paths;
	ASSERT_NO_THROW({ApplicationData appData(paths);});
}

TEST(ApplicationDataTest, OnlyValidPaths)
{
	// Expects to find "Data" correctly ...
	ASSERT_TRUE(boost::filesystem::exists("Data")) <<
			"WARNING: Data directory could not be found, possibly you are running the "
			"test from the wrong directory, some or other following tests will fail. " << std::endl <<
			"FIX THIS TEST FIRST!";

	std::vector<std::string> paths;
	paths.push_back("Data");
	paths.push_back("Data");
	paths.push_back("invalid");
	paths.push_back("invalid");
	paths.push_back("Data/ApplicationDataTest/Directory1");
	// Path with backslashes should get ignored
	paths.push_back("Data\\ApplicationDataTest\\Directory2");

	ApplicationData data(paths);
	ASSERT_EQ(2u, data.getPaths().size());
}

TEST(ApplicationDataTest, GetPathsTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	std::string path1 = "Data/ApplicationDataTest/Directory1";
	std::string path2 = "Data/ApplicationDataTest/Directory2";
	paths.push_back(path1);
	paths.push_back(path2);
	ApplicationData data(paths);

	ASSERT_EQ(2u, data.getPaths().size());

	paths = data.getPaths();

	EXPECT_TRUE(boost::filesystem::equivalent(
					path(path1),
					path(paths[0]))) << path1 << " not considered equivalent to " << paths[0];
	EXPECT_TRUE(boost::filesystem::equivalent(
					path(path2),
					path(paths[1]))) << path2 << " not considered equivalent to " << paths[1];
}

TEST(ApplicationDataTest, FindFileTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest/Directory1");
	paths.push_back("Data/ApplicationDataTest/Directory2");
	ApplicationData data(paths);

	boost::filesystem::path filePath;

	ASSERT_EQ(2u, data.getPaths().size());
	EXPECT_TRUE(fileIsFoundCorrectly(data, "uniqueFile1.txt", "Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data, "uniqueFile2.txt", "Directory2"));
	EXPECT_TRUE(fileIsFoundCorrectly(data, "duplicatedFile.txt", "Directory1"));

	EXPECT_EQ("", data.findFile("missingFile.txt"));
	EXPECT_EQ("", data.findFile(""));
}

TEST(ApplicationDataTest, TryFindFileTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest/Directory1");
	paths.push_back("Data/ApplicationDataTest/Directory2");
	ApplicationData data(paths);

	std::string fileName;
	boost::filesystem::path filePath;

	EXPECT_TRUE(data.tryFindFile("uniqueFile1.txt", &fileName));
	EXPECT_EQ(data.findFile("uniqueFile1.txt"), fileName);

	EXPECT_TRUE(data.tryFindFile("duplicatedFile.txt", &fileName));
	EXPECT_EQ(data.findFile("duplicatedFile.txt"), fileName);

	fileName = "Should Not Change";

	EXPECT_FALSE(data.tryFindFile("missingFile.txt", &fileName));
	EXPECT_EQ("Should Not Change", fileName);
}


TEST(ApplicationDataTest, DirectorywithSpaceTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::string path = "Data/ApplicationDataTest/Test Directory/uniqueFile.txt";
	ApplicationData data(path);

	EXPECT_TRUE(fileIsFoundCorrectly(data, "uniqueFile.txt", "Test Directory"));
}

// We don't accept anything that contains backslashes ...
TEST(ApplicationDataTest, MessedUpSlashesTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	paths.push_back("Data\\ApplicationDataTest/Directory1");
	paths.push_back("Data/ApplicationDataTest\\Directory2");
	ApplicationData data(paths);

	ASSERT_EQ(0u, data.getPaths().size());
}

TEST(ApplicationDataTest, DeepPathTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest");
	ApplicationData data(paths);

	EXPECT_TRUE(fileIsFoundCorrectly(data, "Directory1/uniqueFile1.txt", "Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data, "Directory2/uniqueFile2.txt", "Directory2"));
}

TEST(ApplicationDataTest, InitFromFile)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	ASSERT_ANY_THROW({ApplicationData data("nonexistingFile.xxx");});
	ASSERT_NO_THROW({ApplicationData data("Data/ApplicationDataTest/testFile1.txt");});

	ApplicationData data("Data/ApplicationDataTest/testFile1.txt");

	ASSERT_EQ(2u, data.getPaths().size());
	EXPECT_TRUE(fileIsFoundCorrectly(data, "uniqueFile1.txt", "Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data, "uniqueFile2.txt", "Directory2"));
	EXPECT_TRUE(fileIsFoundCorrectly(data, "duplicatedFile.txt", "Directory1"));
	EXPECT_EQ("", data.findFile("missingFile.txt"));
}

TEST(ApplicationDataTest, IsValidFilenameTest)
{
	ApplicationData data("Data/ApplicationDataTest/testFile1.txt");

	std::string validFileName("123.txt");
	std::string invalidFileName1("");
	std::string invalidFileName2("\\invalid\\123.txt");

	EXPECT_TRUE(data.isValidFilename(validFileName));
	EXPECT_FALSE(data.isValidFilename(invalidFileName1));
	EXPECT_FALSE(data.isValidFilename(invalidFileName2));
}

TEST(ApplicationDataTest, FindDirectory)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest");
	ApplicationData data(paths);

	std::string result;
	EXPECT_TRUE(data.tryFindFile("Directory1", &result));
	EXPECT_TRUE(result.size() != 0);
}