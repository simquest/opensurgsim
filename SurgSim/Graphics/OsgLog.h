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

#ifndef SURGSIM_GRAPHICS_OSGLOG_H
#define SURGSIM_GRAPHICS_OSGLOG_H

#include <osg/Notify>

#include "SurgSim/Framework/Log.h"

namespace SurgSim
{

namespace Graphics
{

/// Enable logging of OSG through SurgSim Logging System
/// To use this, an object of OsgLog class must be created.
/// Then call osg::setNotifyHandler() to let OSG use OSS logging system.
class OsgLog : public osg::NotifyHandler
{
public:
	/// Constructor
	/// If OSS_DEBUG is defined, set OSG's log level to the lowest (osg::DEBUG_FP).
	/// So that all info can be logged.
	/// Otherwise, keep OSG's default log level (osg::NOTICE).

	/// Note that message can still be filtered out in user defined derived method notify().
	OsgLog();



	/// User defined derived log Method
	/// Based on log level 'severity', this method decides whether to log 'message' with OSS logging system.

	/// \param	severity Log level of message to be logged.
	/// \param	message The actual message to be logged.
	void notify(osg::NotifySeverity severity, const char *message) override;

private:
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGLOG_H
