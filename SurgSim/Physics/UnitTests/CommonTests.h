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

/// \file CommonTests.h
/// Common data structure for physics tests

#ifndef SURGSIM_PHYSICS_UNITTESTS_COMMONTESTS_H
#define SURGSIM_PHYSICS_UNITTESTS_COMMONTESTS_H

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/FixedConstraintContact.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/MlcpPhysicsSolution.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidConstraintContact.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

class CommonTests : public ::testing::Test
{
public:
	/// Setup the test case by creating all object
	void SetUp()
	{
		// Set the time step
		dt = 1e-3;

		// Create a fixed world to define constraint against it
		m_fixedWorldRepresentation = std::make_shared<FixedRepresentation>("FixedPlane");
		m_fixedWorldRepresentation->setIsGravityEnabled(false);
		{
			// Simply do 1 time step to make sure things are initialized (compliance matrix...)
			m_fixedWorldRepresentation->beforeUpdate(dt);
			m_fixedWorldRepresentation->update(dt);
			m_fixedWorldRepresentation->afterUpdate(dt);
		}

		// Create the physics manager state
		m_physicsManagerState = std::make_shared<PhysicsManagerState>();

		// Create a Rigid Sphere
		std::shared_ptr<RigidRepresentation> rigidSphereRepresentation;
		rigidSphereRepresentation = std::make_shared<RigidRepresentation>("RigidSphere");
		{
			double radius = 1e-2;
			std::shared_ptr<Shape> shape = std::make_shared<SphereShape>(radius);
			rigidSphereRepresentation->setShape(shape);
			rigidSphereRepresentation->setDensity(1000);
			rigidSphereRepresentation->setIsGravityEnabled(false);
			{
				// Simply do 1 time step to make sure things are initialized (compliance matrix...)
				rigidSphereRepresentation->beforeUpdate(dt);
				rigidSphereRepresentation->update(dt);
				rigidSphereRepresentation->afterUpdate(dt);
			}
			m_allRepresentations.push_back(rigidSphereRepresentation);
		}

		// Create a Rigid Box
		std::shared_ptr<RigidRepresentation> rigidBoxRepresentation = std::make_shared<RigidRepresentation>("RigidBox");
		{
			double size[3]={0.01, 0.02, 0.03};
			rigidBoxRepresentation->setDensity(1000);
			std::shared_ptr<Shape> shape = std::make_shared<BoxShape>(size[0], size[1], size[2]);
			rigidBoxRepresentation->setShape(shape);
			rigidBoxRepresentation->setIsGravityEnabled(false);
			{
				// Simply do 1 time step to make sure things are initialized (compliance matrix...)
				rigidBoxRepresentation->beforeUpdate(dt);
				rigidBoxRepresentation->update(dt);
				rigidBoxRepresentation->afterUpdate(dt);
			}
			m_allRepresentations.push_back(rigidBoxRepresentation);
		}
	}

	void resetMlcpProblem(int nbDof, int nbConstraint)
	{
		if (m_physicsManagerState)
		{
			m_physicsManagerState->getMlcpProblem().A.setZero(nbConstraint, nbConstraint);
			m_physicsManagerState->getMlcpProblem().b.setZero(nbConstraint);
			m_physicsManagerState->getMlcpProblem().CHt.setZero(nbDof, nbConstraint);
			m_physicsManagerState->getMlcpProblem().H.resize(nbConstraint, nbDof);
			m_physicsManagerState->getMlcpProblem().mu.setZero(nbConstraint);
			m_physicsManagerState->getMlcpProblem().constraintTypes.clear();

			m_physicsManagerState->getMlcpSolution().x.setZero(nbConstraint);
			m_physicsManagerState->getMlcpSolution().dofCorrection.setZero(nbDof);
		}
	}

protected:
	/// Time step
	double dt;

	/// Fixed representation to define constraint in fixed space
	std::shared_ptr<Representation> m_fixedWorldRepresentation;

	/// Vector of all representations
	std::vector<std::shared_ptr<Representation>> m_allRepresentations;

	/// Vector of representations useful for the current test
	std::vector<std::shared_ptr<Representation>> m_usedRepresentations;

	/// Vector of constraints useful for the current test
	std::vector<std::shared_ptr<Constraint>> m_usedConstraints;

	/// The unique physics manager state
	std::shared_ptr<PhysicsManagerState> m_physicsManagerState;
};

}; // namespace Physics
}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_UNITTESTS_COMMONTESTS_H
