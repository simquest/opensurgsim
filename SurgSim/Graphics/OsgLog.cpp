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

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Graphics/OsgLog.h>

using SurgSim::Graphics::OsgLog;

void OsgLog::notify(osg::NotifySeverity severity, const char *message)
{
	if (severity <= osg::FATAL)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getLogger("OsgLog"))<<message;
	}
	else if (osg::FATAL < severity && severity <= osg::WARN)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("OsgLog"))<<message;
	}
	else if (osg::WARN< severity && severity <= osg::INFO)
	{
		SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getLogger("OsgLog"))<<message;
	}
	else if (osg::INFO < severity && severity <= osg::DEBUG_FP)
	{
		SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("OsgLog"))<<message;
	}
}