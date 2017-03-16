// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/LinearSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/FemConstraintFixedPoint.h"
#include "SurgSim/Physics/FemConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FemConstraintFrictionalSliding.h"
#include "SurgSim/Physics/FemConstraintFrictionlessContact.h"
#include "SurgSim/Physics/FemConstraintFrictionlessSliding.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"

#include "SurgSim/Physics/Fem3DCorotationalTetrahedronRepresentation.h"

namespace SurgSim
{

namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Component,
				 SurgSim::Physics::Fem3DCorotationalTetrahedronRepresentation,
				 Fem3DCorotationalTetrahedronRepresentation);

Fem3DCorotationalTetrahedronRepresentation::Fem3DCorotationalTetrahedronRepresentation(const std::string& name)
		: Fem3DRepresentation(name)
{
	using SurgSim::Physics::ConstraintImplementation;
	using SurgSim::Physics::FemConstraintFixedPoint;
	using SurgSim::Physics::FemConstraintFixedRotationVector;
	using SurgSim::Physics::FemConstraintFrictionalSliding;
	using SurgSim::Physics::FemConstraintFrictionlessContact;
	using SurgSim::Physics::FemConstraintFrictionlessSliding;

	Fem3DElementCorotationalTetrahedron tetCoro;
	setFemElementType(tetCoro.getClassName());

	setComplianceWarping(true);

	// Register all the constraint for this representation in the ConstraintImplementation factory
	// Because Fem3DCorotationalTetrahedronRepresentation derives from Fem3DRepresentation, it can use the exact
	// same constraint implementation. The constraint expression is exactly the same and the compliance
	// used will be the correct one.
	ConstraintImplementation::getFactory().addImplementation(
			typeid(Fem3DCorotationalTetrahedronRepresentation), std::make_shared<FemConstraintFixedPoint>());
	ConstraintImplementation::getFactory().addImplementation(
			typeid(Fem3DCorotationalTetrahedronRepresentation), std::make_shared<FemConstraintFixedRotationVector>());
	ConstraintImplementation::getFactory().addImplementation(
			typeid(Fem3DCorotationalTetrahedronRepresentation), std::make_shared<FemConstraintFrictionlessContact>());
	ConstraintImplementation::getFactory().addImplementation(
			typeid(Fem3DCorotationalTetrahedronRepresentation), std::make_shared<FemConstraintFrictionlessSliding>());
	ConstraintImplementation::getFactory().addImplementation(
			typeid(Fem3DCorotationalTetrahedronRepresentation), std::make_shared<FemConstraintFrictionalSliding>());
}

Fem3DCorotationalTetrahedronRepresentation::~Fem3DCorotationalTetrahedronRepresentation()
{
}

void Fem3DCorotationalTetrahedronRepresentation::setFemElementType(const std::string& type)
{
	Fem3DElementCorotationalTetrahedron tetCoro;
	SURGSIM_ASSERT(type == tetCoro.getClassName()) <<
												   "Invalid FemElement type found '" << type
												   << "', default and expected is '" << tetCoro.getClassName() << "'";
	Fem3DRepresentation::setFemElementType(type);
}

Math::Matrix Fem3DCorotationalTetrahedronRepresentation::getNodeTransformation(
		const Math::OdeState& state, size_t nodeId)
{
	std::vector<Math::Matrix33d> elementRotations;
	Math::Matrix33d R3x3;

	for (auto const& element : m_femElements)
	{
		auto node = std::find(element->getNodeIds().begin(), element->getNodeIds().end(), nodeId);
		if (node != element->getNodeIds().end())
		{
			elementRotations.push_back(std::static_pointer_cast<Fem3DElementCorotationalTetrahedron>(element)->
					getRotationMatrix());
		}
	}

	SURGSIM_ASSERT(elementRotations.size() > 0) << "Node " << nodeId << " happens to not belong to any FemElements";

	R3x3 = elementRotations[0];
	for (size_t i = 2; i <= elementRotations.size(); i++)
	{
		double ai = 1.0/static_cast<double>(i);
		R3x3 = Eigen::Quaterniond(R3x3).slerp(1.0 - ai, Eigen::Quaterniond(elementRotations[i - 1]));
	}

	return Math::Matrix(R3x3);
}

} // namespace Physics

} // namespace SimQuest
