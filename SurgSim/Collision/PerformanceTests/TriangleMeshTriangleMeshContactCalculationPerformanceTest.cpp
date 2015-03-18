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

#include "SurgSim/Collision/TriangleMeshTriangleMeshDcdContact.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::TriangleMeshPlain;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

void doTriangleMeshTriangleMeshTest(std::shared_ptr<MeshShape> meshA,
	const RigidTransform3d& meshATransform,
	std::shared_ptr<MeshShape> meshB,
	const RigidTransform3d& meshBTransform)
{
}

TEST(TriangleMeshTriangleMeshContactCalculationPerformanceTests, IntersectionTest)
{
	using SurgSim::Math::MeshShape;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::RigidTransform3d;

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto meshA = std::make_shared<MeshShape>();
	meshA->load("MeshShapeData/stapler_collision.ply");

	auto meshB = std::make_shared<MeshShape>();
	meshB->load("MeshShapeData/wound_deformable.ply");

	std::shared_ptr<ShapeCollisionRepresentation> meshARep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 0");
	meshARep->setShape(meshA);

	std::shared_ptr<ShapeCollisionRepresentation> meshBRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 1");
	meshBRep->setShape(meshB);

	TriangleMeshTriangleMeshDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshARep, meshBRep);

	Framework::Timer timer;
	int loops = 100;
	size_t contacts = 0;
	for (int i = 0 ; i < loops; ++i)
	{
		pair->clearContacts();
		meshARep->getCollisions().unsafeGet().clear();
		meshBRep->getCollisions().unsafeGet().clear();
		timer.beginFrame();
		calcContact.calculateContact(pair);
		timer.endFrame();
		contacts += pair->getContacts().size();
	}

	RecordProperty("ContactsPerLoop", boost::to_string(pair->getContacts().size()));
	RecordProperty("DurationPerLoop", boost::to_string(timer.getCumulativeTime() / loops));
	RecordProperty("Loops", boost::to_string(loops));
	RecordProperty("TotalContacts", boost::to_string(contacts));
	RecordProperty("TotalDuration", boost::to_string(timer.getCumulativeTime()));
}

}
}
