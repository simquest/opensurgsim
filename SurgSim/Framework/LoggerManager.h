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

#include <SurgSim/Framework/Logger.h>

namespace SurgSim
{
namespace Framework
{

/// Class to safely handle access to a group of loggers, manipulate the global
/// logging threshold, and fetch logger(s) from a global pool
class LoggerManager
{
public:
	/// Constructor
	explicit LoggerManager();
	~LoggerManager();

	/// Sets default output.
	/// \param	output	The output class to be used.
	void setDefaultOutput(std::shared_ptr<LogOutput> output);

	/// Return the default output
	std::shared_ptr<LogOutput> getDefaultOutput() const;


	/// Gets the default logger
	/// \return	The default logger.
	std::shared_ptr<Logger> getDefaultLogger();

	/// Creates a logger that logs to the standard error output.
	/// \param name The name to use for the logger.
	/// \return The new logger.
	std::shared_ptr<Logger> createConsoleLogger(const std::string& name);


	/// Sets a logger with a given name, overwriting the old logger
	/// \param	name  	The name of logger.
	/// \param	logger	The logger.
	void setLogger(std::string name, std::shared_ptr<Logger> logger);

	/// Gets a logger with a given name, creates a new one if none exists  or the logger
	/// has been deallocated.
	/// \param	name	The name.
	/// \return	The logger.
	std::shared_ptr<Logger> getLogger(const std::string& name);


	/// Sets a threshold for all the loggers.
	/// \param	threshold	The threshold.
	void setThreshold(int threshold);

	/// Sets a threshold for a subgroup of loggers, the group is chosen by finding all loggers
	/// whose pathname starts with the same string as the pathname given here.
	/// \param	path	 	Full pathname of the file.
	/// \param	threshold	The threshold to use for these loggers.
	void setThreshold(const std::string& path, int threshold);

	/// Return the threshold used by all loggers
	/// \return Threshold used by all the loggers.
	int getThreshold() const;

private:

	/// Search for a logger with given name and return it.
	/// If not found, nulptr will be returned
	/// \param	name	The name of the logger to be found.
	/// \return	Logger with given name if found; Otherwise, nulptr
	std::shared_ptr<Logger> lookUpLogger(const std::string& name);

	/// Create a logger with given name and output
	/// \param	name	The name of the logger to be created.
	/// \param	output	Output class to be used by the logger.
	std::shared_ptr<Logger> createLogger(const std::string& name, std::shared_ptr<LogOutput> output);

	/// Keep track of all the loggers
	std::unordered_map<std::string, std::shared_ptr<Logger>> m_loggers;

	/// Use for default output of the logger
	std::shared_ptr<LogOutput> m_defaultOutput;

	/// Threshold used by all loggers
	int m_globalThreshold;

	boost::mutex m_mutex;
};

}; // Framework
}; // SurgSim

#endif //SURGSIM_FRAMEWORK_LOGGERMANAGER_H