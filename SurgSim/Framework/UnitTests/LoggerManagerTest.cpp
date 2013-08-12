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
/// Tests for the SURGSIM_LOG_*() macros and the related classes.

#include <gtest/gtest.h>
#include <SurgSim/Framework/Log.h>

#include <string>

using SurgSim::Framework::Logger;
using SurgSim::Framework::LoggerManager;
using SurgSim::Framework::StreamOutput;

class MockOutput : public SurgSim::Framework::LogOutput
{
public:
	MockOutput()
	{
	}

	~MockOutput()
	{
	}

	bool writeMessage(const std::string& message)
	{
		logMessage = message;
		return true;
	}
	void reset()
	{
		logMessage.clear();
	}

	std::string logMessage;
};


TEST(LoggerManagerTest, Constructor)
{
	EXPECT_NO_THROW({std::shared_ptr<LoggerManager> loggerManager(new LoggerManager());});
}

TEST(LoggerManagerTest, defaultOutput)
{
	std::shared_ptr<LoggerManager> loggerManager(std::make_shared<LoggerManager>());
	std::shared_ptr<StreamOutput> output(std::make_shared<StreamOutput>(std::cerr));
	loggerManager->setDefaultOutput(output);

	EXPECT_EQ(output, loggerManager->getDefaultOutput());
}

TEST(LoggerManagerTest, defaultLoggerTest)
{
	std::shared_ptr<LoggerManager> loggerManager(std::make_shared<LoggerManager>());
	std::shared_ptr<Logger> defaultLogger(loggerManager->getDefaultLogger());

	EXPECT_TRUE(nullptr != defaultLogger);
	EXPECT_EQ(loggerManager->getDefaultOutput(), defaultLogger->getOutput());
	EXPECT_EQ(loggerManager->getThreshold(), defaultLogger->getThreshold());
}


TEST(LoggerManagerTest, getLogger)
{
	std::shared_ptr<LoggerManager> loggerManager(std::make_shared<LoggerManager>());

	std::shared_ptr<Logger> retrieved(loggerManager->getLogger("test"));

	EXPECT_TRUE(nullptr != retrieved);
}

TEST(LoggerManagerTest, threshold)
{
	std::shared_ptr<LoggerManager> loggerManager(std::make_shared<LoggerManager>());
	loggerManager->setThreshold(SurgSim::Framework::LOG_LEVEL_CRITICAL);
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_CRITICAL, loggerManager->getThreshold());

	std::shared_ptr<Logger> logger1(loggerManager->getLogger("logger1"));
	std::shared_ptr<Logger> logger2(loggerManager->getLogger("logger2"));
	std::shared_ptr<Logger> testLogger(loggerManager->getLogger("testLogger"));

	loggerManager->setThreshold(SurgSim::Framework::LOG_LEVEL_INFO);
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_INFO, logger1->getThreshold());
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_INFO, logger2->getThreshold());
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_INFO, testLogger->getThreshold());

	loggerManager->setThreshold("logger", SurgSim::Framework::LOG_LEVEL_WARNING);
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_WARNING, logger1->getThreshold());
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_WARNING, logger2->getThreshold());
	EXPECT_NE(SurgSim::Framework::LOG_LEVEL_WARNING, testLogger->getThreshold());
}