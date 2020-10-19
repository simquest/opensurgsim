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

#ifndef SURGSIM_FRAMEWORK_ASSERTMESSAGE_H
#define SURGSIM_FRAMEWORK_ASSERTMESSAGE_H

#include <memory>

#include "SurgSim/Framework/LogMessageBase.h"


namespace SurgSim
{
namespace Framework
{


/// An exception class thrown by SURGSIM_ASSERT() failures and SURGSIM_FAILURE().
/// \ingroup assertAPI
class AssertionFailure : public std::runtime_error
{
public:
	/// Constructor
	/// \param message Exception message
	explicit AssertionFailure(const std::string& message) : std::runtime_error(message)
	{
	}
};


/// An internal message class used for assertion failures.  Dies after logging.
/// \ingroup assertInternals
class AssertMessage : public LogMessageBase
{
public:
	/// The type used for the callback function that is triggered after an assertion has failed.
	typedef void (*DeathCallback)(const std::string& message);

	/// Constructor.
	/// \param logger %Logger used to log this message.
	explicit AssertMessage(Logger* logger) : LogMessageBase(logger, LOG_LEVEL_CRITICAL)
	{
	}

	/// Constructor.
	/// \param logger %Logger used to log this message.
	explicit AssertMessage(const std::unique_ptr<Logger>& logger) : LogMessageBase(logger.get(), LOG_LEVEL_CRITICAL)
	{
	}

	/// Constructor.
	/// \param logger %Logger used to log this message.
	explicit AssertMessage(const std::shared_ptr<Logger>& logger) : LogMessageBase(logger.get(), LOG_LEVEL_CRITICAL)
	{
	}

	/// Destructor, which may throw an exception if the failure behavior does
	// As per https://docs.microsoft.com/en-us/cpp/cpp/exception-specifications-throw-cpp?view=vs-2019
	// noexcept(false) has implementation from Visual Studio 2017 15.5
#if defined(_MSC_VER) && _MSC_VER < 1912
	~AssertMessage() throw(...) 
#else
	~AssertMessage() noexcept(false)
#endif
	{
		flush();
		m_killMeNow(getMessage());
	}

	/// After an assertion has failed, call some arbitrary function.
	/// The callback function should cause the application (or at least the current thread) to terminate.
	///
	/// Thread-unsafe if called concurrently from multiple threads, or concurrently with a failing assertion.
	static void setFailureCallback(DeathCallback callback);

	/// Get the callback that will currently be called after an assertion has failed.
	/// Thread-unsafe if called concurrently from multiple threads, or concurrently with a failing assertion.
	/// \return The callback.
	static DeathCallback getFailureCallback();

	/// After an assertion has failed, throw a C++ exception.
	/// Thread-unsafe if called concurrently from multiple threads, or concurrently with a failing assertion.
	static void setFailureBehaviorToThrow()
	{
		setFailureCallback(throwException);
	}

	/// After an assertion has failed, enter the debugger or kill the application in a system-dependent way.
	/// Thread-unsafe if called concurrently from multiple threads, or concurrently with a failing assertion.
	static void setFailureBehaviorToDeath()
	{
		setFailureCallback(killApplication);
	}

private:
	/// Kill the application by throwing an exception.
	/// \param errorMessage Message describing the error.
	static void throwException(const std::string& errorMessage);

	/// Enter the debugger or kill the application in a system-dependent way.
	/// \param errorMessage Message describing the error (which will be ignored).
	static void killApplication(const std::string& errorMessage);


	/// The callback function that is triggered after an assertion has failed.
	/// Thread-unsafe if called concurrently from multiple threads.
	static DeathCallback m_killMeNow;
};


};  // namespace Framework
};  // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_ASSERTMESSAGE_H
