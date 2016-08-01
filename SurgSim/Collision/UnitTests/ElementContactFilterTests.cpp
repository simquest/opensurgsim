// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include <gmock/gmock.h>

#include <memory>

#include "SurgSim/Collision/ElementContactFilter.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/SegmentMeshShape.h"

namespace
{

std::shared_ptr<SurgSim::Collision::CollisionPair> makePair()
{
	auto rep1 = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rep1");
	rep1->setShape(std::make_shared<SurgSim::Math::MeshShape>());

	auto rep2 = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rep2");
	rep2->setShape(std::make_shared<SurgSim::Math::SegmentMeshShape>());

	return std::make_shared <SurgSim::Collision::CollisionPair>(rep1, rep2);
}


std::shared_ptr<SurgSim::Collision::Contact> makeContact(
	size_t triMesh1, size_t elementMesh1,
	size_t triMesh2, size_t elementMesh2)
{
	std::pair<SurgSim::DataStructures::Location, SurgSim::DataStructures::Location> penetrationPoints;

	penetrationPoints.first.triangleMeshLocalCoordinate =
		SurgSim::DataStructures::IndexedLocalCoordinate(triMesh1, SurgSim::Math::Vector3d::Zero());
	penetrationPoints.first.elementMeshLocalCoordinate =
		SurgSim::DataStructures::IndexedLocalCoordinate(elementMesh1, SurgSim::Math::Vector3d::Zero());

	penetrationPoints.second.triangleMeshLocalCoordinate =
		SurgSim::DataStructures::IndexedLocalCoordinate(triMesh2, SurgSim::Math::Vector3d::Zero());
	penetrationPoints.second.elementMeshLocalCoordinate =
		SurgSim::DataStructures::IndexedLocalCoordinate(elementMesh2, SurgSim::Math::Vector3d::Zero());

	return std::make_shared<SurgSim::Collision::Contact>(
			   SurgSim::Collision::COLLISION_DETECTION_TYPE_DISCRETE, 0.0, 1.0,
			   SurgSim::Math::Vector3d::Zero(), SurgSim::Math::Vector3d::Zero(), penetrationPoints);

}



}




namespace SurgSim
{
namespace Collision
{
class ElementContactFilterTest : public testing::Test
{

public:
	void SetUp()
	{
		filter = std::make_shared<ElementContactFilter>("filter");
		pair = makePair();
	}

	std::shared_ptr<Physics::PhysicsManagerState> state;
	std::shared_ptr<ElementContactFilter> filter;
	std::shared_ptr<SurgSim::Collision::CollisionPair> pair;
};


TEST_F(ElementContactFilterTest, Accessors)
{
	std::vector<size_t> ignores(1, 1);

}

TEST_F(ElementContactFilterTest, Noop)
{
	EXPECT_NO_THROW(filter->filterContacts(state, pair));
	std::vector<size_t> ignores(1, 1);

	filter->setFilterElements(pair->getRepresentations().first, ignores);
	EXPECT_NO_THROW(filter->filterContacts(state, pair));

	pair->addContact(makeContact(0, 0, 0, 0));
	EXPECT_NO_THROW(filter->filterContacts(state, pair));
}

TEST_F(ElementContactFilterTest, RemoveOnTriangleMesh)
{

	std::vector<size_t> ignores;
	ignores.push_back(0);
	ignores.push_back(2);
	filter->setFilterElements(pair->getRepresentations().first, ignores);

	pair->addContact(makeContact(2, 10, 10, 10));
	pair->addContact(makeContact(0, 10, 10, 10));
	pair->addContact(makeContact(1, 10, 10, 10));
	pair->addContact(makeContact(1, 10, 10, 10));
	pair->addContact(makeContact(2, 10, 10, 10));
	pair->addContact(makeContact(2, 10, 10, 10));

	EXPECT_EQ(6u, pair->getContacts().size());
	EXPECT_NO_THROW(filter->filterContacts(state, pair));
	EXPECT_EQ(2u, pair->getContacts().size());
}

TEST_F(ElementContactFilterTest, RemoveOnTriangleMeshSwapped)
{
	std::vector<size_t> ignores;
	ignores.push_back(0);
	ignores.push_back(2);
	filter->setFilterElements(pair->getRepresentations().first, ignores);

	pair->swapRepresentations();

	pair->addContact(makeContact(10, 10, 2, 10));
	pair->addContact(makeContact(10, 10, 0, 10));
	pair->addContact(makeContact(10, 10, 1, 10));
	pair->addContact(makeContact(10, 10, 1, 10));
	pair->addContact(makeContact(10, 10, 2, 10));
	pair->addContact(makeContact(10, 10, 2, 10));


	EXPECT_EQ(6u, pair->getContacts().size());
	EXPECT_NO_THROW(filter->filterContacts(state, pair));
	EXPECT_EQ(2u, pair->getContacts().size());
}

TEST_F(ElementContactFilterTest, RemoveOnSegmentMesh)
{
	std::vector<size_t> ignores;
	ignores.push_back(0);
	ignores.push_back(2);
	filter->setFilterElements(pair->getRepresentations().second, ignores);
	auto type = pair->getRepresentations().second->getShapeType();

	pair->addContact(makeContact(10, 10, 10, 0));
	pair->addContact(makeContact(10, 10, 10, 1));
	pair->addContact(makeContact(10, 10, 10, 1));
	pair->addContact(makeContact(10, 10, 10, 2));
	pair->addContact(makeContact(10, 10, 10, 2));
	pair->addContact(makeContact(10, 10, 10, 2));

	EXPECT_EQ(6u, pair->getContacts().size());
	filter->filterContacts(state, pair);
	EXPECT_EQ(2u, pair->getContacts().size());
}

TEST_F(ElementContactFilterTest, FilterOnBoth)
{
	{
		std::vector<size_t> ignores;
		ignores.push_back(0);
		filter->setFilterElements(pair->getRepresentations().second, ignores);
	}
	{
		std::vector<size_t> ignores;
		ignores.push_back(100);
		filter->setFilterElements(pair->getRepresentations().first, ignores);
	}
	auto type = pair->getRepresentations().second->getShapeType();

	pair->addContact(makeContact(101, 10, 10, 2)); // Don't filter
	pair->addContact(makeContact(100, 10, 10, 0)); // Filter due to both
	pair->addContact(makeContact(100, 10, 10, 1)); // Filter due to index 100 on first shape
	pair->addContact(makeContact(100, 10, 10, 2)); // Filter due to index 100 on first shape
	pair->addContact(makeContact(100, 10, 10, 0)); // Filter due to both
	pair->addContact(makeContact(101, 10, 10, 0)); // Filter due to index 0 on second shape
	pair->addContact(makeContact(102, 10, 10, 3)); // Don't filter

	EXPECT_EQ(7u, pair->getContacts().size());
	filter->filterContacts(state, pair);
	EXPECT_EQ(2u, pair->getContacts().size());
}



}
}
