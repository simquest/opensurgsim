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

#include "SurgSim/Physics/PhysicsConvert.h"

#include "SurgSim/Physics/RigidRepresentationState.h"

namespace YAML
{

Node convert<SurgSim::Physics::RigidRepresentationState>::encode(const SurgSim::Physics::RigidRepresentationState& rhs)
{
	YAML::Node data(rhs.encode());

	YAML::Node result;
	result[rhs.getClassName()] = data;

	return result;
}

bool convert<SurgSim::Physics::RigidRepresentationState>::decode(
	const Node& node,
	SurgSim::Physics::RigidRepresentationState& rhs) //NOLINT
{
	bool result = false;
	if (node[rhs.getClassName()].IsDefined())
	{
		YAML::Node data;
		data = node[rhs.getClassName()];
		rhs.decode(data);
		result = true;
	}
	return result;
}

} // namespace YAML
