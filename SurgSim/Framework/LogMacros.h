// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest LLC.
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

#ifndef SURGSIM_FRAMEWORK_LOG_MACROS_H
#define SURGSIM_FRAMEWORK_LOG_MACROS_H

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Framework/LogMessage.h"

namespace SurgSim
{
namespace Framework
{

/// Converts a short \c level name to the log level enum value.
/// \param level Short log level name (DEBUG, INFO, WARNING, SEVERE, or CRITICAL)
/// \return Log level enum value
#define SURGSIM_LOG_LEVEL(level) ::SurgSim::Framework::LOG_LEVEL_ ## level

/// Logs a message to the specified \c logger with the short \c level name.
/// \param logger Logger used to log the message
/// \param level Level of this log message (DEBUG, INFO, WARNING, SEVERE, or CRITICAL)
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG(logger, WARNING) << messageText;
#define SURGSIM_LOG(logger, level) \
	if (SURGSIM_LOG_LEVEL(level) < (logger)->getThreshold()) \
	{ \
	} \
	else \
		/* important: no curly braces around this! */ \
		::SurgSim::Framework::LogMessage((logger), SURGSIM_LOG_LEVEL(level))

/// Logs a message to the specified \c logger at the DEBUG level.
/// \param logger Logger used to log the message
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_DEBUG(logger) << messageText;
#define SURGSIM_LOG_DEBUG(logger) SURGSIM_LOG(logger, DEBUG)
/// Logs a message to the specified \c logger at the INFO level.
/// \param logger Logger used to log the message
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_INFO(logger) << messageText;
#define SURGSIM_LOG_INFO(logger) SURGSIM_LOG(logger, INFO)
/// Logs a message to the specified \c logger at the WARNING level.
/// \param logger Logger used to log the message
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_WARNING(logger) << messageText;
#define SURGSIM_LOG_WARNING(logger) SURGSIM_LOG(logger, WARNING)
/// Logs a message to the specified \c logger at the SEVERE level.
/// \param logger Logger used to log the message
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_SEVERE(logger) << messageText;
#define SURGSIM_LOG_SEVERE(logger) SURGSIM_LOG(logger, SEVERE)
/// Logs a message to the specified \c logger at the CRITICAL level.
/// \param logger Logger used to log the message
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_CRITICAL(logger) << messageText;
#define SURGSIM_LOG_CRITICAL(logger) SURGSIM_LOG(logger, CRITICAL)


/// Logs a message to the specified \c logger with the short \c level name if \c condition is true.
/// \param condition Condition to test
/// \param logger Logger used to log the message
/// \param level Level of this log message (DEBUG, INFO, WARNING, SEVERE, or CRITICAL)
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_IF(size >= 100, logger, INFO) << "size is " << size;
#define SURGSIM_LOG_IF(condition, logger, level) \
	if (! (condition)) \
	{ \
	} \
	else \
		/* important: no curly braces around this! */ \
		SURGSIM_LOG(logger, level)

/// Two macros are needed to make sure the argument is fully expanded.
#define SURGSIM_FLAG_VARIABLE_NAME_HELPER(base, line) base ## line
#define SURGSIM_FLAG_VARIABLE_NAME(base, line)  SURGSIM_FLAG_VARIABLE_NAME_HELPER(base, line)
/// Define a variable name that depends on the line number in the source file where the macro is called from.
/// We need this because we can't just add braces to create a scope; that would break using << to add more info.
#define SURGSIM_LOG_ONCE_VARIABLE  SURGSIM_FLAG_VARIABLE_NAME(surgsimLogOnceFlag, __LINE__)

/// Logs a message to the specified \c logger with the short \c level name, but only the first time this statement
/// is reached.
/// \param condition Condition to test
/// \param logger Logger used to log the message
/// \param level Level of this log message (DEBUG, INFO, WARNING, SEVERE, or CRITICAL)
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_ONCE(logger, level) << failText;
#define SURGSIM_LOG_ONCE(logger, level) \
	static int SURGSIM_LOG_ONCE_VARIABLE = 0; \
	if ((SURGSIM_LOG_ONCE_VARIABLE++) != 0) \
	{ \
	} \
	else \
		/* important: no curly braces around this! */ \
		SURGSIM_LOG(logger, level)


/// Logs a message to the specified \c logger with the short \c level name if \c condition is true, but only the
/// first time *this* particular condition is true.
/// \param condition Condition to test
/// \param logger Logger used to log the message
/// \param level Level of this log message (DEBUG, INFO, WARNING, SEVERE, or CRITICAL)
/// \return Stream to output the log message
///
/// Example:
///   SURGSIM_LOG_ONCE_IF(condition, logger, level) << failText;
#define SURGSIM_LOG_ONCE_IF(condition, logger, level) \
	static int SURGSIM_LOG_ONCE_VARIABLE = 0; \
	if (! (condition)) \
	{ \
	} \
	else if ((SURGSIM_LOG_ONCE_VARIABLE++) != 0) \
	{ \
	} \
	else \
		/* important: no curly braces around this! */ \
		SURGSIM_LOG(logger, level)

}; // namespace Framework
}; // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_LOG_MACROS_H
