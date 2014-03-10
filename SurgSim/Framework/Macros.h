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

/// GCC macro to write out an _Pragma statement inside a macro, disabled for other platforms
#ifdef __GNUC__
#define SURGSIM_DO_PRAGMA(x) _Pragma (#x)
#else
#define SURGSIM_DO_PRAGMA(x)
#endif

///@{
/// Set of macros to create a unique name with a common basename
#define SURGSIM_CONCATENATE_DETAIL(x, y) x##y
#define SURGSIM_CONCATENATE(x, y) SURGSIM_CONCATENATE_DETAIL(x, y)
#define SURGSIM_MAKE_UNIQUE(x) SURGSIM_CONCATENATE(x, __COUNTER__)
///@}

#endif
