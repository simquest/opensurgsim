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

#ifndef SURGSIM_FRAMEWORK_LOGGER_H
#define SURGSIM_FRAMEWORK_LOGGER_H

#include "LogOutput.h"

#include <string>
#include <sstream>
#include <memory>

namespace SurgSim
{
namespace Framework
{

/// Logging levels, use these
/// DEBUG Use at your discretion
/// INFO informational, notify of state changes
/// WARNING something failed, but the impact of the failure is not know or minimal (e.g. purely visual)
/// SEVERE something and it will impact functionality, some parts of the program will not function correctly
/// CRITICAL Used by assertion, after using this level the program will not be functional at all
enum LogLevel
{
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_SEVERE,
    LOG_LEVEL_CRITICAL
};

/// Logger class, takes a name and an LogOutput class, which determines in what
/// way the logger presents its output. Construct at the beginning and pass to
/// the the macro to use it.
class Logger
{
public:
	Logger(const std::string& name, std::shared_ptr<LogOutput> output) :
		m_threshold(LOG_LEVEL_DEBUG), // include all logging levels
		m_name(name),
		m_output(output)
	{
	}

	~Logger()
	{
	}

	/// \return a default logger to be used by all components
	static Logger* getDefaultLogger();

	/// Uses the contained instance of LogOutput to write the log message
	/// \return true on success
	/// \param message the message to be printed
	bool writeMessage(const std::string& message)
	{
		return m_output->writeMessage(message);
	}

	/// The threshold used for logging, anything message with less than this level
	/// will be ignored
	int getThreshold() const
	{
		return m_threshold;
	}

	/// Sets the logging threshold.
	/// \param val The value for the threshold.
	void setThreshold(int val)
	{
		m_threshold = val;
	}

	/// Get the output object used by this logger.
	/// \return the current output object used this logger.
	std::shared_ptr<LogOutput> getOutput() const
	{
		return m_output;
	}

	/// Set the output object used by this logger.
	/// \param val The output object to be used.
	void setOutput(std::shared_ptr<LogOutput> val)
	{
		m_output = val;
	}

	/// This logger's name.
	std::string getName() const
	{
		return m_name;
	}

private:
	int m_threshold;
	std::string m_name;
	std::shared_ptr<LogOutput> m_output;
};

}; // namespace Framework
}; // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_LOGGER_H
