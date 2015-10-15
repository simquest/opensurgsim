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

#ifndef SURGSIM_FRAMEWORK_LOGGERMANAGER_H
#define SURGSIM_FRAMEWORK_LOGGERMANAGER_H

#include <unordered_map>
#include <boost/thread/mutex.hpp>

namespace SurgSim
{
namespace Framework
{

class Logger;
class LogOutput;

/// Class to safely handle access to a group of loggers, manipulate the global
/// logging threshold, and fetch logger(s) from a global pool
class LoggerManager
{
public:
	/// Constructor
	LoggerManager();

	/// Destructor
	~LoggerManager();

	/// Sets/Changes default output.
	/// \param	output	The output class to be used.
	void setDefaultOutput(std::shared_ptr<LogOutput> output);

	/// Return the default output
	std::shared_ptr<LogOutput> getDefaultOutput() const;

	/// Gets the default logger
	/// \return	The default logger.
	std::shared_ptr<Logger> getDefaultLogger();

	/// Gets a logger with a given name, creates a new one if none exists  or the logger
	/// has been deallocated.
	/// \param	name	The name.
	/// \return	The logger.
	std::shared_ptr<Logger> getLogger(const std::string& name);

	/// Sets a threshold for all loggers.
	/// \param	threshold	The threshold.
	void setThreshold(int threshold);

	/// Sets a threshold for a subgroup of loggers, the group is chosen by finding all loggers
	/// whose pathname starts with the same string as the pathname given.
	/// \param	path	 	Full pathname of the file.
	/// \param	threshold	The threshold to use for these loggers.
	void setThreshold(const std::string& path, int threshold);

	/// Return the threshold used by all loggers
	/// \return Threshold used by all the loggers.
	int getThreshold() const;

private:
	/// Keep track of all the loggers
	std::unordered_map<std::string, std::shared_ptr<Logger>> m_loggers;

	/// Keep track of subgroup thresholds.
	std::vector<std::pair<std::string, int>> m_thresholds;

	/// Use for default output of the logger
	std::shared_ptr<LogOutput> m_defaultOutput;

	/// Threshold used by all loggers
	int m_globalThreshold;

	boost::mutex m_mutex;

	// Aug 13, 2013
	// VS2012 does not support "delete" now
	LoggerManager(const LoggerManager&);
	LoggerManager& operator=(const LoggerManager&);
};

}; // Framework
}; // SurgSim

#endif //SURGSIM_FRAMEWORK_LOGGERMANAGER_H