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

#include <gmock/gmock.h>

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Framework/LogOutput.h"

int main(int argc, char** argv)
{
	//Disable logging during tests
	std::shared_ptr<SurgSim::Framework::LoggerManager> loggerManager;
	loggerManager = SurgSim::Framework::Logger::getLoggerManager();
	loggerManager->setDefaultOutput(std::make_shared<SurgSim::Framework::NullOutput>());

	testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
