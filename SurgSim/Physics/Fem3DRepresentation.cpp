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

#include <SurgSim/Framework/Assert.h>

#include <SurgSim/Physics/Fem3DRepresentation.h>

namespace
{
void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform,
										Vector* x, bool rotationOnly = false)
{
	typedef Vector::Index IndexType;

	IndexType numNodes = x->size() / 3;
	SURGSIM_ASSERT(numNodes * 3 == x->size()) <<
		"Unexpected number of dof in a Fem3D state vector (not a multiple of 3)";

	for (IndexType nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::Vector3d xi = SurgSim::Math::getSubVector(*x, nodeId, 3);
		SurgSim::Math::Vector3d xiTransformed;
		if (rotationOnly)
		{
			xiTransformed = transform.linear() * xi;
		}
		else
		{
			xiTransformed = transform * xi;
		}
		SurgSim::Math::setSubVector(xiTransformed, nodeId, 3, x);
	}
}
}

namespace SurgSim
{

namespace Physics
{

Fem3DRepresentation::Fem3DRepresentation(const std::string& name) :
	FemRepresentation(name)
{
	// Reminder: m_numDofPerNode is held by DeformableRepresentation
	// but needs to be set by all concrete derived classes
	m_numDofPerNode = 3;
}

Fem3DRepresentation::~Fem3DRepresentation()
{
}

RepresentationType Fem3DRepresentation::getType() const
{
	return REPRESENTATION_TYPE_FEM3D;
}

void Fem3DRepresentation::transformState(std::shared_ptr<DeformableRepresentationState> state,
	const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
	transformVectorByBlockOf3(transform, &state->getAccelerations(), true);
}

} // namespace Physics

} // namespace SurgSim
