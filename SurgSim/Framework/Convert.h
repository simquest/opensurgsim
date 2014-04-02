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

#ifndef SURGSIM_FRAMEWORK_CONVERT_H
#define SURGSIM_FRAMEWORK_CONVERT_H

#include <yaml-cpp/yaml.h>
#include "SurgSim/Framework/Log.h"

/// \note HS-2013-dec-23 The gcc and msvc compilers seem to have different requirements when a template class
///       needs to be passed template parameters in a specialization, that extend the original template interface
///       gcc needs the template<> statement before the new template parameters, msvc does not like it at all.
#ifdef _GNUC_
#define SURGSIM_DOUBLE_SPECIALIZATION template<>
#else
#define SURGSIM_DOUBLE_SPECIALIZATION
#endif


namespace SurgSim
{
namespace Serialize
{
	/// Logger name for Serialization
	const std::string serializeLogger = "Serialization";

};
};

#endif // SURGSIM_FRAMEWORK_CONVERT_H