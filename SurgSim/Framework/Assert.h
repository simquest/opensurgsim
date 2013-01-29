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

#ifndef SURGSIM_FRAMEWORK_ASSERT_H
#define SURGSIM_FRAMEWORK_ASSERT_H

#include <SurgSim/Framework/Logger.h>
#include <SurgSim/Framework/AssertMessage.h>

namespace SurgSim
{
namespace Framework
{

#if !defined(SURGSIM_ASSERT_LOGGER)
/// Logger used to log asserts.
/// The default logger is used if no other logger is defined.
#define SURGSIM_ASSERT_LOGGER  ::SurgSim::Framework::Logger::getDefaultLogger()
#endif

// Helper macro to determine the function name currently being compiled.
// (I wish we didn't need this garbage, but...)
#if defined(__func__)
#define SURGSIM_CURRENT_FUNCTION __func__
#elif defined(__FUNCSIG__)
#define SURGSIM_CURRENT_FUNCTION __FUNCSIG__
#elif defined(__PRETTY_FUNCTION__)
#define SURGSIM_CURRENT_FUNCTION __PRETTY_FUNCTION__
#elif defined(__FUNCTION__)
#define SURGSIM_CURRENT_FUNCTION __FUNCTION__
#else
#define SURGSIM_CURRENT_FUNCTION "???"
#endif

/// Helper macros to turn its argument into a quoted string constant.
/// \param x Argument to convert into a string
/// \return Quoted string constant.  Note that macros such as __LINE__ may be quoted literally rather than expanded.
#define SURGSIM_MAKE_STRING(x)  #x

/// Assert that \c condition is true.  If not, abort program execution, printing an error message that will include
/// \c failText, the condition, source file/line, etc.
/// \param condition Condition to test
/// \return Stream to output extra assert information
///
/// Example:
///   SURGSIM_ASSERT(index >= 0) << "bad index: " << index;
#define SURGSIM_ASSERT(condition) \
	if ((condition)) \
	{ \
	} \
	else \
		/* important: no curly braces around this! */ \
		::SurgSim::Framework::AssertMessage(SURGSIM_ASSERT_LOGGER) << "*** Assertion failed: " << \
			SURGSIM_MAKE_STRING(condition) << " ***" << std::endl << \
			"    in " << SURGSIM_CURRENT_FUNCTION << std::endl << \
			"    at " << __FILE__ << ":" << __LINE__ << std::endl

/// Report that something very bad has happened and abort program execution.  An error message will be printed, and will
/// include \c failText, source file/line, etc.
/// This is pretty similar to SURGSIM_ASSERT(true, ...), except "true" won't be included in the resulting message. =)
/// \return Stream to output extra failure information
///
/// Example:
///   SURGSIM_FAILURE() << failText;
#define SURGSIM_FAILURE() \
	::SurgSim::Framework::AssertMessage(SURGSIM_ASSERT_LOGGER) << "*** Failure ***" << std::endl << \
	"    in " << SURGSIM_CURRENT_FUNCTION << std::endl << \
	"    at " << __FILE__ << ":" << __LINE__ << std::endl

};  // namespace Framework
};  // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_ASSERT_H
