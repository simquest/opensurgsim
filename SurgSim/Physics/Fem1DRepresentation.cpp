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

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/LinearSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentationPlyReaderDelegate.h"

namespace
{

void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform, SurgSim::Math::Vector* x,
							   bool rotationOnly = false)
{
	typedef SurgSim::Math::Vector::Index IndexType;

	IndexType numNodes = x->size() / 6;

	SURGSIM_ASSERT(numNodes * 6 == x->size())
		<< "Unexpected number of dof in a Fem1D state vector (not a multiple of 6)";

	for (IndexType nodeId = 0; nodeId < numNodes; nodeId++)
	{
		// Only the translational dof are transformed, rotational dof remains unchanged
		SurgSim::Math::Vector3d xi = x->segment<3>(6 * nodeId);

		x->segment<3>(6 * nodeId) = (rotationOnly) ? transform.linear() * xi : transform * xi;
	}
}

}

namespace SurgSim
{

namespace Physics
{

Fem1DRepresentation::Fem1DRepresentation(const std::string& name) : FemRepresentation(name)
{
	// Reminder: m_numDofPerNode is held by DeformableRepresentation but needs to be set by all concrete derived classes
	m_numDofPerNode = 6;
}

Fem1DRepresentation::~Fem1DRepresentation()
{
}

RepresentationType Fem1DRepresentation::getType() const
{
	return REPRESENTATION_TYPE_FEM1D;
}

bool Fem1DRepresentation::doWakeUp()
{
	using SurgSim::Math::LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix;

	if (!FemRepresentation::doWakeUp())
	{
		return false;
	}

	// Make use of a specialized linear solver for symmetric tri-diagonal block matrix of block size 6
	m_odeSolver->setLinearSolver(std::make_shared<LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<6>>());

	return true;
}

void Fem1DRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
										 const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

bool Fem1DRepresentation::doLoadFile(std::shared_ptr<SurgSim::DataStructures::PlyReader> reader)
{
	auto thisAsSharedPtr = std::static_pointer_cast<Fem1DRepresentation>(getSharedPtr());
	auto readerDelegate = std::make_shared<Fem1DRepresentationPlyReaderDelegate>(thisAsSharedPtr);

	bool result = true;
	if (reader->setDelegate(readerDelegate))
	{
		// PlyReader::parseFile loads the fem into the shared_ptr passed to the readerDelegate constructor.
		reader->parseFile();
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__ <<
			"File " << m_filename << " is not an acceptable PLY.";
		result = false;
	}

	return result;
}

} // namespace Physics

} // namespace SurgSim
