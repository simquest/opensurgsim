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
/// Tests for LoggerManager class.

#include "SurgSim/Framework/Log.h"

#include <string>
#include <memory>

#include <gtest/gtest.h>

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

TEST(LoggerManagerTest, DefaultOutput)
{
	auto loggerManager = std::make_shared<LoggerManager>();
	auto output = std::make_shared<StreamOutput>(std::cerr);
	loggerManager->setDefaultOutput(output);

	EXPECT_EQ(output, loggerManager->getDefaultOutput());
}

TEST(LoggerManagerTest, DefaultLoggerTest)
{
	auto loggerManager = std::make_shared<LoggerManager>();
	auto defaultLogger = loggerManager->getDefaultLogger();

	EXPECT_TRUE(nullptr != defaultLogger);
	EXPECT_EQ(loggerManager->getDefaultOutput(), defaultLogger->getOutput());
	EXPECT_EQ(loggerManager->getThreshold(), defaultLogger->getThreshold());
}


TEST(LoggerManagerTest, GetLogger)
{
	auto loggerManager = std::make_shared<LoggerManager>();

	/// getLogger() will create a new logger if a logger with given name is not found
	auto retrieved = loggerManager->getLogger("test");
	EXPECT_TRUE(nullptr != retrieved);

	/// getLogger() guarantees to return a logger with given name
	{
		auto temp = loggerManager->getLogger("logger");
	}
	EXPECT_TRUE(nullptr != loggerManager->getLogger("logger"));
}

TEST(LoggerManagerTest, Threshold)
{
	auto loggerManager = std::make_shared<LoggerManager>();

	/// Check default log level
	auto logger0 = loggerManager->getLogger("logger0");
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_WARNING, logger0->getThreshold());

	/// setThreshold(level) will change log level of existing loggers
	/// and the default log level
	loggerManager->setThreshold(SurgSim::Framework::LOG_LEVEL_CRITICAL);
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_CRITICAL, loggerManager->getThreshold());
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_CRITICAL, logger0->getThreshold());

	/// setThreshold(level) will affect log level of newly created loggers
	auto logger1 = loggerManager->getLogger("logger1");
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_CRITICAL, logger1->getThreshold());


	auto logger2 = loggerManager->getLogger("logger2");
	auto logger3 = loggerManager->getLogger("logger3");
	auto testLogger = loggerManager->getLogger("testLogger");

	/// setThreshold(pattern, level) will change log level of existing loggers
	/// which match given name pattern
	/// Logger with different pattern will not be changed
	/// Default log level will not be affected
	loggerManager->setThreshold("logger", SurgSim::Framework::LOG_LEVEL_WARNING);
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_WARNING, logger2->getThreshold());
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_WARNING, logger3->getThreshold());
	EXPECT_NE(SurgSim::Framework::LOG_LEVEL_WARNING, testLogger->getThreshold());
	EXPECT_NE(SurgSim::Framework::LOG_LEVEL_WARNING, loggerManager->getThreshold());

	/// setThreshold(pattern, level) will affect newly created loggers if the logger's name matches the pattern
	auto testLogger2 = loggerManager->getLogger("testLogger2");
	EXPECT_NE(SurgSim::Framework::LOG_LEVEL_WARNING, testLogger2->getThreshold());
	auto logger5 = loggerManager->getLogger("logger5");
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_WARNING, logger5->getThreshold());

	/// Logger manager should own the logger.
	loggerManager->getLogger("xxx")->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
	EXPECT_EQ(SurgSim::Framework::LOG_LEVEL_DEBUG, loggerManager->getLogger("xxx")->getThreshold());
}