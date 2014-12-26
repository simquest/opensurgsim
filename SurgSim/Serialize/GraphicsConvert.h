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

#ifndef SURGSIM_SERIALIZE_GRAPHICSCONVERT_H
#define SURGSIM_SERIALIZE_GRAPHICSCONVERT_H

#include "SurgSim/Serialize/Convert.h"
#include "SurgSim/Serialize/FrameworkConvert.h"
#include "SurgSim/Serialize/MathConvert.h"

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/SphereRepresentation.h"

namespace YAML
{

/// Specialize of YAML::convert<> template Represensation class.
template <>
struct convert <SurgSim::Graphics::Representation>
{
	static Node encode(const SurgSim::Graphics::Representation& rhs);
	static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::Representation> rhs);

};

/// Specialize of YAML::convert<> template SphereRepresensation class.
template <>
struct convert <SurgSim::Graphics::SphereRepresentation>
{
	static Node encode(const SurgSim::Graphics::SphereRepresentation& rhs);
	static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs);
};

};

#endif // SURGSIM_SERIALIZE_GRAPHICSCONVERT_H
