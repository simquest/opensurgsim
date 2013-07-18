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

/// \file Simple Test for PreUpdate calculation

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include <SurgSim/Physics/PhysicsManagerState.h>
#include <SurgSim/Physics/PreUpdate.h>
#include <SurgSim/Physics/Representation.h>

#include <SurgSim/Physics/UnitTests/MockObjects.h>

namespace SurgSim
{
namespace Physics
{


TEST(PreUpdateTest, UpdateCallTest)
{
	std::shared_ptr<MockRepresentation> mockRepresentation = std::make_shared<MockRepresentation>();
	std::shared_ptr<PhysicsManagerState> physicsManagerState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<Representation>> allRepresentations;

	allRepresentations.push_back(mockRepresentation);
	physicsManagerState->setRepresentations(allRepresentations);

	double dt = 1e-3;
	std::shared_ptr<PreUpdate> preUpdateComputation = std::make_shared<PreUpdate>();
	EXPECT_EQ(0, mockRepresentation->getPreUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getPostUpdateCount());
	preUpdateComputation->update(dt, physicsManagerState);
	EXPECT_EQ(1, mockRepresentation->getPreUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getPostUpdateCount());
	preUpdateComputation->update(dt, physicsManagerState);
	EXPECT_EQ(2, mockRepresentation->getPreUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getPostUpdateCount());
	preUpdateComputation->update(dt, physicsManagerState);
	EXPECT_EQ(3, mockRepresentation->getPreUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getUpdateCount());
	EXPECT_EQ(0, mockRepresentation->getPostUpdateCount());
}

}; // namespace Physics
}; // namespace SurgSim

