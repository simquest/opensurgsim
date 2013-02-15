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

#include <SurgSim/Framework/ApplicationData.h>

#include <boost/filesystem.hpp>

using SurgSim::Framework::ApplicationData;
using boost::filesystem::path;

::testing::AssertionResult isContained(std::string expected, std::vector<std::string> argument)
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

::std::ostream& operator<<(::std::ostream& os, const std::vector<std::string>& content) {
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

::testing::AssertionResult fileIsFoundCorrectly(
	const ApplicationData& data,
	const std::string& searchFileName,
	const std::string& expectedDirectoryName) 
{

	// First file should be there and in directory1
	std::string fileName = data.findFile(searchFileName);
	boost::filesystem::path filePath(fileName);

	if (! boost::filesystem::exists(filePath))
	{
		return ::testing::AssertionFailure() << "Result does not exist " << fileName;
	}
	if (! filePath.is_absolute())
	{
		return ::testing::AssertionFailure() << "Result not absolute path " << fileName <<
			" expected " << boost::filesystem::canonical(filePath);
	}
	if (fileName != filePath.make_preferred().string())
	{
		return ::testing::AssertionFailure() << "Result not system format path " << fileName << 
			" expected " << filePath.make_preferred().string();
	}
	if (boost::filesystem::path(searchFileName).filename() != filePath.filename() )
	{
		return ::testing::AssertionFailure() << "Expected " << searchFileName << 
			" Result " << filePath.filename();
	}
	if ( fileName.find(expectedDirectoryName) == -1)
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
	std::vector<std::string> paths;
	paths.push_back("Data");
	paths.push_back("Data");
	paths.push_back("xxx");
	ApplicationData data(paths);

	EXPECT_EQ(1, data.getPaths().size());
}

TEST(ApplicationDataTest, GetPathsTest)
{
	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest/Directory1");
	paths.push_back("Data\\ApplicationDataTest\\Directory2");
	ApplicationData data (paths);

	EXPECT_EQ(2, data.getPaths().size());
	
	paths = data.getPaths();

	EXPECT_TRUE(boost::filesystem::equivalent(
		path("Data/ApplicationDataTest/Directory1"),
		path(paths[0])));
	EXPECT_TRUE(boost::filesystem::equivalent(
		path("Data/ApplicationDataTest/Directory2"),
		path(paths[1])));
}

TEST (ApplicationDataTest, FindFileTest)
{
	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest/Directory1");
	paths.push_back("Data\\ApplicationDataTest\\Directory2");
	ApplicationData data (paths);
	
	std::string fileName;
	boost::filesystem::path filePath;

	EXPECT_EQ(2, data.getPaths().size());
	EXPECT_TRUE(fileIsFoundCorrectly(data,"uniqueFile1.txt","Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"uniqueFile2.txt","Directory2"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"duplicatedFile.txt","Directory1"));

	EXPECT_EQ("", data.findFile("missingFile.txt"));
}

TEST (ApplicationDataTest, MessedUpSlashesTest)
{
	std::vector<std::string> paths;
	paths.push_back("Data\\ApplicationDataTest/Directory1");
	paths.push_back("Data/ApplicationDataTest\\Directory2");
	ApplicationData data(paths);

	EXPECT_EQ(2, data.getPaths().size());
	EXPECT_TRUE(fileIsFoundCorrectly(data,"uniqueFile1.txt","Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"uniqueFile2.txt","Directory2"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"duplicatedFile.txt","Directory1"));
	EXPECT_EQ("", data.findFile("missingFile.txt"));
}

TEST (ApplicationDataTest, DeepPathTest)
{
	std::vector<std::string> paths;
	paths.push_back("Data/ApplicationDataTest");
	ApplicationData data(paths);

	EXPECT_TRUE(fileIsFoundCorrectly(data,"Directory1/uniqueFile1.txt","Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"Directory2\\uniqueFile2.txt","Directory2"));
}

TEST (ApplicationDataTest, InitFromFile)
{
	ASSERT_ANY_THROW({ApplicationData data("nonexistingFile.xxx");});
	ASSERT_NO_THROW({ApplicationData data("Data/ApplicationDataTest/testFile1.txt");});

	ApplicationData data("Data/ApplicationDataTest/testFile1.txt");

	EXPECT_EQ(2, data.getPaths().size());
	EXPECT_TRUE(fileIsFoundCorrectly(data,"uniqueFile1.txt","Directory1"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"uniqueFile2.txt","Directory2"));
	EXPECT_TRUE(fileIsFoundCorrectly(data,"duplicatedFile.txt","Directory1"));
	EXPECT_EQ("", data.findFile("missingFile.txt"));

}