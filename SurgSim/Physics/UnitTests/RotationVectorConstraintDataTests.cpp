// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Physics/RotationVectorConstraintData.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Physics::Fem1DElementBeam;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RotationVectorRigidFem1DConstraintData;

TEST(RotationVectorConstraintDataTests, TestSetGet)
{
	using SurgSim::Framework::AssertionFailure;

	RotationVectorRigidFem1DConstraintData rotationVectorConstraintData;
	auto Id = SurgSim::Math::Matrix33d::Identity();

	EXPECT_THROW(rotationVectorConstraintData.getCurrentRotationVector(), AssertionFailure);

	auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("rigid");
	auto rigidRAtGrasp = rigid->getPose().linear();
	EXPECT_THROW(rotationVectorConstraintData.setRigidOrFixedRotation(nullptr, Id), AssertionFailure);
	rotationVectorConstraintData.setRigidOrFixedRotation(rigid, rigidRAtGrasp);

	EXPECT_THROW(rotationVectorConstraintData.getCurrentRotationVector(), AssertionFailure);

	EXPECT_THROW(rotationVectorConstraintData.setFem1DRotation(nullptr, 0), AssertionFailure);
	auto fem1d = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("fem1d");
	EXPECT_THROW(rotationVectorConstraintData.setFem1DRotation(fem1d, 0), AssertionFailure);

	auto initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(6, 2);
	initialState->getPositions().setZero();
	initialState->getPositions().segment<3>(6) = SurgSim::Math::Vector3d(1.0, 0.0, 0.0);
	fem1d->setInitialState(initialState);
	auto elementData = std::make_shared<SurgSim::Physics::FemElementStructs::FemElement1DParameter>();
	elementData->enableShear = false;
	elementData->massDensity = 950;
	elementData->nodeIds.push_back(0);
	elementData->nodeIds.push_back(1);
	elementData->poissonRatio = 0.35;
	elementData->radius = 0.01;
	elementData->youngModulus = 1e6;
	std::shared_ptr<SurgSim::Physics::Fem1DElementBeam> element(new SurgSim::Physics::Fem1DElementBeam(elementData));
	fem1d->addFemElement(element);
	fem1d->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	EXPECT_NO_THROW(rotationVectorConstraintData.setFem1DRotation(fem1d, 0));

	EXPECT_NO_THROW(rotationVectorConstraintData.getCurrentRotationVector());
}
