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
#include "SurgSim/Framework/Assert.h"


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
		logMessage = "";
	}

	std::string logMessage;
};

class AssertTest : public ::testing::Test
{
public:
	void SetUp()
	{
		logOutput = std::make_shared<MockOutput>();
		testLogger = SurgSim::Framework::Logger::getLogger("test");
		testLogger->setOutput(logOutput);
		// testLogger will be used for assertions in most tests, due to the definition of SURGSIM_ASSERT_LOGGER below.
		savedCallback = SurgSim::Framework::AssertMessage::getFailureCallback();
	}

	void TearDown()
	{
		SurgSim::Framework::AssertMessage::setFailureCallback(savedCallback);
		testLogger.reset();
		logOutput.reset();
	}

	std::shared_ptr<MockOutput> logOutput;
	std::shared_ptr<SurgSim::Framework::Logger> testLogger;
	SurgSim::Framework::AssertMessage::DeathCallback savedCallback = nullptr;
};

typedef AssertTest AssertDeathTest;


TEST_F(AssertTest, DefaultAssertLogger)
{
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "extra information would go here", SurgSim::Framework::AssertionFailure);
}

#undef SURGSIM_ASSERT_LOGGER   // override the default definition
#define SURGSIM_ASSERT_LOGGER testLogger   // defined in the test fixture, above

inline bool stringContains(const std::string& string, const std::string& fragment)
{
	return string.find(fragment) != std::string::npos;
}

static int numIgnoredAssertions = 0;

static void ignoreAssertionKeepGoing(const std::string& message)
{
	++numIgnoredAssertions;
}

TEST_F(AssertTest, Assertion)
{
	logOutput->reset();
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "extra information would go here", SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(stringContains(logOutput->logMessage, "1 == 2")) <<
		"message: '" << logOutput->logMessage << "'";
	EXPECT_TRUE(stringContains(logOutput->logMessage, "AssertTest.cpp")) <<
		"message: '" << logOutput->logMessage << "'";
	EXPECT_TRUE(stringContains(logOutput->logMessage, "extra information")) <<
		"message: '" << logOutput->logMessage << "'";

	logOutput->reset();
	EXPECT_NO_THROW({SURGSIM_ASSERT(3 == 3) << "extra information would go here";});
	EXPECT_EQ("", logOutput->logMessage) <<
		"message: '" << logOutput->logMessage << "'";
}

TEST_F(AssertTest, Failure)
{
	logOutput->reset();
	EXPECT_THROW(SURGSIM_FAILURE() << "extra information would go here", SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(stringContains(logOutput->logMessage, "AssertTest.cpp")) <<
		"message: '" << logOutput->logMessage << "'";
	EXPECT_TRUE(stringContains(logOutput->logMessage, "extra information")) <<
		"message: '" << logOutput->logMessage << "'";
}

TEST_F(AssertTest, Manipulators)
{
	logOutput->reset();
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "aAa" << std::endl << "bBb", SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(stringContains(logOutput->logMessage, "aAa\nbBb") ||
				stringContains(logOutput->logMessage, "aAa\r\n\bBb")) <<
		"message: '" << logOutput->logMessage << "'";

	logOutput->reset();
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "[" << std::hex << std::setw(5) << std::setfill('0') << 0x1234 << "]",
				 SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(stringContains(logOutput->logMessage, "[01234]")) <<
		"message: '" << logOutput->logMessage << "'";

	logOutput->reset();
	// The next message should not show any evidence of previous manipulators.
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "[" << 987 << "]", SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(stringContains(logOutput->logMessage, "[987]")) <<
		"message: '" << logOutput->logMessage << "'";
}

TEST_F(AssertTest, ExceptionBehavior)
{
	logOutput->reset();

	SurgSim::Framework::AssertMessage::setFailureBehaviorToThrow();
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "extra information would go here", SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW({SURGSIM_ASSERT(3 == 3) << "extra information would go here";});

	EXPECT_THROW(SURGSIM_FAILURE() << "extra information would go here", SurgSim::Framework::AssertionFailure);
}

TEST_F(AssertTest, Callback)
{
	logOutput->reset();

	typedef SurgSim::Framework::AssertMessage::DeathCallback CallbackType;
	const CallbackType defaultCallback = SurgSim::Framework::AssertMessage::getFailureCallback();

	SurgSim::Framework::AssertMessage::setFailureBehaviorToThrow();
	const CallbackType throwCallback = SurgSim::Framework::AssertMessage::getFailureCallback();
	EXPECT_EQ(throwCallback, defaultCallback);

	SurgSim::Framework::AssertMessage::setFailureCallback(ignoreAssertionKeepGoing);
	EXPECT_EQ(static_cast<CallbackType>(ignoreAssertionKeepGoing),
			  SurgSim::Framework::AssertMessage::getFailureCallback());
	numIgnoredAssertions = 0;
	EXPECT_NO_THROW(SURGSIM_ASSERT(1 == 2) << "extra information would go here");
	EXPECT_EQ(1, numIgnoredAssertions);
	EXPECT_NO_THROW({SURGSIM_ASSERT(3 == 3) << "extra information would go here";});
	EXPECT_EQ(1, numIgnoredAssertions);
	EXPECT_NO_THROW(SURGSIM_FAILURE() << "extra information would go here");
	EXPECT_EQ(2, numIgnoredAssertions);

	SurgSim::Framework::AssertMessage::setFailureBehaviorToThrow();
	EXPECT_EQ(throwCallback, SurgSim::Framework::AssertMessage::getFailureCallback());
	EXPECT_THROW(SURGSIM_ASSERT(1 == 2) << "extra information would go here", SurgSim::Framework::AssertionFailure);
	EXPECT_EQ(2, numIgnoredAssertions);
}

TEST_F(AssertDeathTest, DebuggerBehaviorAssertFailed)
{
	logOutput->reset();
	SurgSim::Framework::AssertMessage::setFailureBehaviorToDeath();
	// The assertion should die with no output to stdout.
	ASSERT_DEATH_IF_SUPPORTED({SURGSIM_ASSERT(1 == 2) << "extra information would go here";}, "^$");
}

TEST_F(AssertDeathTest, DebuggerBehaviorAssertSucceded)
{
	logOutput->reset();
	SurgSim::Framework::AssertMessage::setFailureBehaviorToDeath();
	EXPECT_NO_THROW({SURGSIM_ASSERT(3 == 3) << "extra information would go here";});
}

TEST_F(AssertDeathTest, DebuggerBehaviorFailure)
{
	logOutput->reset();
	SurgSim::Framework::AssertMessage::setFailureBehaviorToDeath();
	// The assertion should die with no output to stdout.
	ASSERT_DEATH_IF_SUPPORTED({SURGSIM_FAILURE() << "extra information would go here";}, "^$");
}
