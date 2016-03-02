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

#ifndef SURGSIM_FRAMEWORK_MACROS_H
#define SURGSIM_FRAMEWORK_MACROS_H

/// Declare the class name of a class with the appropriate function header,
/// do not use quotes
#define SURGSIM_CLASSNAME(ClassName) \
	virtual std::string getClassName() const {return #ClassName;}

/// Macro to tell GCC this is a used variable, and not to optimize it out
#ifdef __GNUC__
#define SURGSIM_USED_VARIABLE(x) x __attribute__((used))
#else
#define SURGSIM_USED_VARIABLE(x) x
#endif

///@{
/// Set of macros to create a unique name with a common basename
#define SURGSIM_CONCATENATE_DETAIL(x, y) x##y
#define SURGSIM_CONCATENATE(x, y) SURGSIM_CONCATENATE_DETAIL(x, y)
#define SURGSIM_MAKE_UNIQUE(x) SURGSIM_CONCATENATE(x, __COUNTER__)
///@}

/// \note HS-2013-dec-23 The gcc and msvc compilers seem to have different requirements when a template class
///       needs to be passed template parameters in a specialization, that extend the original template interface
///       gcc needs the template<> statement before the new template parameters, msvc does not like it at all.
#ifdef _GNUC_
#define SURGSIM_DOUBLE_SPECIALIZATION template<>
#else
#define SURGSIM_DOUBLE_SPECIALIZATION
#endif

#define SURGSIM_CLASS(x) #x

#endif
