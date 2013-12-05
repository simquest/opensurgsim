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

#include "SurgSim/Graphics/OsgLog.h"

using SurgSim::Graphics::OsgLog;

namespace SurgSim
{

namespace Graphics
{

OsgLog::OsgLog() : m_logger(SurgSim::Framework::Logger::getLogger("Osg"))
{
#ifdef OSS_DEBUG
	osg::setNotifyLevel(osg::DEBUG_FP);
#endif
}

void OsgLog::notify(osg::NotifySeverity severity, const char *message)
{
	// Map osg logging levels into OSS logging levels
	if (severity <= osg::FATAL)
	{
		SURGSIM_LOG(m_logger, CRITICAL) << message;
	}
	else if (osg::FATAL < severity && severity <= osg::WARN)
	{
		SURGSIM_LOG(m_logger, WARNING) << message;
	}
	else if (osg::WARN < severity && severity <= osg::INFO)
	{
		SURGSIM_LOG(m_logger, INFO) << message;
	}
	else if (osg::INFO < severity && severity <= osg::DEBUG_FP)
	{
		SURGSIM_LOG(m_logger, DEBUG) << message;
	}
	else
	{
		SURGSIM_LOG(m_logger, CRITICAL) << "Unknown severity in OsgLog::notify()";
	}
}

};  // namespace Graphics

};  // namespace SurgSim