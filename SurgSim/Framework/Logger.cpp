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

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Framework/LogOutput.h"

#include <iostream>


namespace SurgSim
{
namespace Framework
{

Logger* Logger::getDefaultLogger()
{
	// Using a static variable has two problems: one, it's technically thread-unsafe; and two, the memory can
	// never be reclaimed.  But it's the simplest thing to do for now.
	static std::shared_ptr<Logger> logger(createConsoleLogger("default"));
	return logger.get();
}

std::shared_ptr<Logger> Logger::createConsoleLogger(const std::string& name)
{
	// Using a static variable has two problems: one, it's technically thread-unsafe; and two, the memory can
	// never be reclaimed.  But it's the simplest thing to do for now.
	static std::shared_ptr<StreamOutput> output(std::make_shared<StreamOutput>(std::cerr));
	return std::make_shared<Logger>(name, output);
}

}
}

