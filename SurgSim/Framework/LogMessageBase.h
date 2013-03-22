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

#ifndef SURGSIM_FRAMEWORK_LOG_MESSAGE_BASE_H
#define SURGSIM_FRAMEWORK_LOG_MESSAGE_BASE_H

#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>

#include "SurgSim/Framework/Logger.h"

namespace SurgSim
{
namespace Framework
{

/// \addtogroup logInternals
/// @{


/// LogMessageBase is a base class to be used to customize messages for logging
/// textual information can be put into a log message by using the << operator
/// in general the message class will output all of its information when the
/// destructor is being invoked, formats the incoming message to timestamp it
/// and adds information about the logger
class LogMessageBase
{
public:

	/// Construct a LogMessage
	/// \param logger The logger to be used
	/// \param level The logging level for this message
	LogMessageBase(Logger* logger, int level);

	/// Destructor.
	~LogMessageBase() {};

	/// Add the given input to the current log message.
	/// \param input The input to be added to the current stream
	template <typename T>
	LogMessageBase& operator <<(T&& input)
	{
		m_stream << input;
		return *this;
	}

	// A specialization for output manipulators (functions that apply to the stream).
	// Otherwise overloaded manipulators like std::endl and std::endl don't work, since the compiler can't know
	// what overloaded variant to apply.
	LogMessageBase& operator <<(std::ios_base& (*manipulator)(std::ios_base&))
	{
		m_stream << *manipulator;
		return *this;
	}

	// A specialization for output manipulators (functions that apply to the stream).
	// Otherwise overloaded manipulators like std::hex and std::endl don't work, since the compiler can't know
	// what overloaded variant to apply.
	LogMessageBase& operator <<(std::ostream& (*manipulator)(std::ostream&))
	{
		m_stream << *manipulator;
		return *this;
	}

protected:
	/// \return the current content of the message to be logged
	std::string getMessage()
	{
		return m_stream.str();
	}

	/// write the current message to the logger
	void flush()
	{
		m_logger->writeMessage(m_stream.str());
	}

private:
	std::ostringstream m_stream;
	Logger* m_logger;
};


/// @}

}; // namespace Framework
}; // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_LOG_MESSAGE_BASE_H
