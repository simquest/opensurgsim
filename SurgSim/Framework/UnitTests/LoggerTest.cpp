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

/** @file
 * Tests for the SURGSIM_LOG_*() macros and the related classes.
 */

#include <gtest/gtest.h>
#include <SurgSim/Framework/Log.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <string>

using SurgSim::Framework::Logger;
using SurgSim::Framework::FileOutput;

::testing::AssertionResult isContained(std::string expected, std::string argument)
{
	if (std::string::npos != argument.find(expected))
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << expected << " not contained in " << argument;
	}
}

class MockOutput : public SurgSim::Framework::LogOutput
{
public:
	MockOutput() {};
	~MockOutput() {};

	bool writeMessage(const std::string& message)
	{
		logMessage = message;
		return true;
	}
	void reset()
	{
		logMessage = "";
	}

	std::string logMessage;
};
TEST(LoggerTest, InitTest)
{
	EXPECT_NO_THROW({Logger("TestLogger", std::make_shared<MockOutput>());});
}

TEST(LoggerTest, MessageTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));

	logger.writeMessage("TestMessage");
	EXPECT_EQ("TestMessage", output->logMessage);
}

TEST(LoggerTest, LogMacroTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));
	logger.setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	output->reset();
	SURGSIM_LOG(&logger, CRITICAL) << "Test Text";
	EXPECT_TRUE(isContained("Test Text",output->logMessage));

	output->reset();
	SURGSIM_LOG(&logger, DEBUG) << "Missing Text";
	EXPECT_EQ("", output->logMessage);

	output->reset();
	SURGSIM_LOG(&logger, WARNING) << "Exactly At Threshold";
	EXPECT_TRUE(isContained("Exactly At Threshold", output->logMessage));
}

TEST(LoggerTest, UniquePtrTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	std::unique_ptr<Logger> logger(new Logger("TestLogger", output));
	logger->setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	output->reset();
	SURGSIM_LOG(logger, CRITICAL) << "Test Text";
	EXPECT_TRUE(isContained("Test Text", output->logMessage));

	output->reset();
	SURGSIM_LOG(logger, DEBUG) << "Missing Text";
	EXPECT_TRUE(isContained("", output->logMessage));

	output->reset();
	SURGSIM_LOG(logger, WARNING) << "Exactly At Threshold";
	EXPECT_TRUE(isContained("Exactly At Threshold", output->logMessage));
}

TEST(LoggerTest, LevelSpecificMacroTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));
	logger.setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	output->reset();
	SURGSIM_LOG_CRITICAL(&logger) << "Test Text";
	EXPECT_TRUE(isContained("Test Text", output->logMessage));

	output->reset();
	SURGSIM_LOG_DEBUG(&logger) << "Missing Text";
	EXPECT_EQ("", output->logMessage);

	output->reset();
	SURGSIM_LOG_WARNING(&logger) << "Exactly At Threshold";
	EXPECT_TRUE(isContained("Exactly At Threshold", output->logMessage));
}

TEST(LoggerTest, IfMacroTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));
	logger.setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	output->reset();
	SURGSIM_LOG_IF(true, &logger, CRITICAL) << "Test Text";
	EXPECT_TRUE(isContained("Test Text", output->logMessage));
	output->reset();
	SURGSIM_LOG_IF(false, &logger, CRITICAL) << "Test Text";
	EXPECT_EQ("", output->logMessage);

	output->reset();
	SURGSIM_LOG_IF(true, &logger, DEBUG) << "Missing Text";
	EXPECT_EQ("", output->logMessage);
	output->reset();
	SURGSIM_LOG_IF(false, &logger, DEBUG) << "Missing Text";
	EXPECT_EQ("", output->logMessage);

	output->reset();
	SURGSIM_LOG_IF(true, &logger, WARNING) << "Exactly At Threshold";
	EXPECT_TRUE(isContained("Exactly At Threshold", output->logMessage));
	output->reset();
	SURGSIM_LOG_IF(false, &logger, WARNING) << "Exactly At Threshold";
	EXPECT_EQ("", output->logMessage);
}

TEST(LoggerTest, OnceMacroTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));
	logger.setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	for (int i = 0;  i < 3; ++i)
	{
		output->reset();
		SURGSIM_LOG_ONCE(&logger, CRITICAL) << "Test Text";
		if (i == 0)
		{
			EXPECT_TRUE(isContained("Test Text", output->logMessage));
		}
		else
		{
			EXPECT_EQ("", output->logMessage);
		}

		output->reset();
		SURGSIM_LOG_ONCE(&logger, DEBUG) << "Missing Text";
		EXPECT_EQ("", output->logMessage);

		output->reset();
		SURGSIM_LOG_ONCE(&logger, CRITICAL) << "More Text";
		if (i == 0)
		{
			EXPECT_TRUE(isContained("More Text", output->logMessage));
		}
		else
		{
			EXPECT_EQ("", output->logMessage);
		}
	}
}

TEST(LoggerTest, OnceIfMacroTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));
	logger.setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	for (int i = 0;  i < 3; ++i)
	{
		output->reset();
		SURGSIM_LOG_ONCE_IF(i >= 0, &logger, CRITICAL) << "Test Text";
		if (i == 0)
		{
			EXPECT_TRUE(isContained("Test Text", output->logMessage));
		}
		else
		{
			EXPECT_EQ("", output->logMessage);
		}

		output->reset();
		SURGSIM_LOG_ONCE_IF(i >= 1, &logger, DEBUG) << "Missing Text";
		EXPECT_EQ("", output->logMessage);

		output->reset();
		SURGSIM_LOG_ONCE_IF(i >= 2, &logger, CRITICAL) << "More Text";
		if (i == 2)
		{
			EXPECT_TRUE(isContained("More Text", output->logMessage));
		}
		else
		{
			EXPECT_EQ("", output->logMessage);
		}
	}
}

TEST(LoggerTest, EndOfLineTest)
{
	std::shared_ptr<MockOutput> output = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));
	logger.setThreshold(SurgSim::Framework::LOG_LEVEL_WARNING);

	output->reset();
	SURGSIM_LOG_CRITICAL(&logger) << "foo" << std::endl << "bar";
	EXPECT_TRUE(isContained("foo\nbar", output->logMessage));
}

TEST(LoggerTest, LogLevelTest)
{
	std::shared_ptr<MockOutput> output	 = std::make_shared<MockOutput>();
	Logger logger(Logger("TestLogger", output));

	output->reset();
	SURGSIM_LOG_INFO(&logger) << "a message";
	EXPECT_TRUE(isContained("INFO", output->logMessage));

	output->reset();
	SURGSIM_LOG_DEBUG(&logger) << "a message";
	EXPECT_TRUE(isContained("DEBUG", output->logMessage));

	output->reset();
	SURGSIM_LOG_WARNING(&logger) << "a message";
	EXPECT_TRUE(isContained("WARNING", output->logMessage));

	output->reset();
	SURGSIM_LOG_SEVERE(&logger) << "a message";
	EXPECT_TRUE(isContained("SEVERE", output->logMessage));

	output->reset();
	SURGSIM_LOG_CRITICAL(&logger) << "a message";
	EXPECT_TRUE(isContained("CRITICAL", output->logMessage));
}

class FileOutputTest : public ::testing::Test
{
public:
	void SetUp()
	{
		output = std::make_shared<FileOutput>("test.log");
	}

	void TearDown()
	{

		output = nullptr;
		boost::filesystem::remove("test.log");
	}

	static std::shared_ptr<FileOutput> output;

};

std::shared_ptr<FileOutput> FileOutputTest::output;

TEST_F(FileOutputTest, BasicTest)
{
	EXPECT_TRUE(boost::filesystem::exists("test.log"));
}

TEST_F(FileOutputTest, CreationFailure)
{
	ASSERT_ANY_THROW(FileOutput("asdfasdf/test.log"));
}

TEST_F(FileOutputTest, Write)
{
	output->writeMessage("TestMessage");
	output = nullptr;

	std::ifstream stream("test.log");
	EXPECT_FALSE(stream.fail());

	std::string message;
	stream >> message;

	EXPECT_EQ("TestMessage",message);
}
