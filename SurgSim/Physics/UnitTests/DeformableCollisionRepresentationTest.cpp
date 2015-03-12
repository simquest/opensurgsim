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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace
{
const double epsilon = 1e-10;
}

namespace SurgSim
{
namespace Physics
{

struct DeformableCollisionRepresentationTest : public ::testing::Test
{
	void SetUp()
	{
		m_runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		m_filename = std::string("Geometry/wound_deformable.ply");
		m_meshShape = std::make_shared<SurgSim::Math::MeshShape>();
		m_meshShape->load(m_filename);
		m_deformableRepresentation = std::make_shared<MockDeformableRepresentation>("DeformableRepresentation");
		m_deformableCollisionRepresentation =
			std::make_shared<DeformableCollisionRepresentation>("DeformableCollisionRepresentation");
	}

	std::shared_ptr<SurgSim::Framework::Runtime> m_runtime;
	std::string m_filename;
	std::shared_ptr<SurgSim::Math::MeshShape> m_meshShape;
	std::shared_ptr<SurgSim::Physics::DeformableRepresentation> m_deformableRepresentation;
	std::shared_ptr<SurgSim::Physics::DeformableCollisionRepresentation> m_deformableCollisionRepresentation;
};

TEST_F(DeformableCollisionRepresentationTest, InitTest)
{
	EXPECT_NO_THROW(DeformableCollisionRepresentation("TestDeformableCollisionRepresentation"));
}

TEST_F(DeformableCollisionRepresentationTest, SetGetDeformableRepresentationTest)
{
	ASSERT_NO_THROW(m_deformableCollisionRepresentation->setDeformableRepresentation(m_deformableRepresentation));
	EXPECT_EQ(m_deformableRepresentation, m_deformableCollisionRepresentation->getDeformableRepresentation());
}

TEST_F(DeformableCollisionRepresentationTest, ShapeTest)
{
	EXPECT_ANY_THROW(m_deformableCollisionRepresentation->getShapeType());
	m_deformableCollisionRepresentation->setShape(m_meshShape);
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_MESH, m_deformableCollisionRepresentation->getShapeType());

	auto meshShape =
		std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(m_deformableCollisionRepresentation->getShape());
	EXPECT_NEAR(m_meshShape->getVolume(), meshShape->getVolume(), epsilon);
	EXPECT_TRUE(m_meshShape->getCenter().isApprox(meshShape->getCenter()));
	EXPECT_TRUE(m_meshShape->getSecondMomentOfVolume().isApprox(meshShape->getSecondMomentOfVolume()));
}

TEST_F(DeformableCollisionRepresentationTest, MeshTest)
{
	m_deformableCollisionRepresentation->setShape(m_meshShape);
	auto actualShape = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(
			m_deformableCollisionRepresentation->getShape());
	ASSERT_NE(nullptr, actualShape);
	EXPECT_EQ(m_meshShape->getNumVertices(), actualShape->getNumVertices());
	EXPECT_EQ(m_meshShape->getNumEdges(), actualShape->getNumEdges());
	EXPECT_EQ(m_meshShape->getNumTriangles(), actualShape->getNumTriangles());
}

TEST_F(DeformableCollisionRepresentationTest, SerializationTest)
{
	auto shape = std::dynamic_pointer_cast<SurgSim::Math::Shape>(m_meshShape);
	m_deformableCollisionRepresentation->setValue("Shape", shape);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*m_deformableCollisionRepresentation));

	std::shared_ptr<SurgSim::Physics::DeformableCollisionRepresentation> newDeformableCollisionRepresentation;
	ASSERT_NO_THROW(newDeformableCollisionRepresentation =
						std::dynamic_pointer_cast<SurgSim::Physics::DeformableCollisionRepresentation>
						(node.as<std::shared_ptr<SurgSim::Framework::Component>>())
				   );

	auto fem3DRepresentation = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("Fem3DRepresentation");
	fem3DRepresentation->setCollisionRepresentation(newDeformableCollisionRepresentation);
	newDeformableCollisionRepresentation->initialize(m_runtime);

	auto mesh = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(newDeformableCollisionRepresentation->getShape());
	EXPECT_NEAR(m_meshShape->getVolume(), mesh->getVolume(), epsilon);
	EXPECT_TRUE(m_meshShape->getCenter().isApprox(mesh->getCenter()));
	EXPECT_TRUE(m_meshShape->getSecondMomentOfVolume().isApprox(mesh->getSecondMomentOfVolume()));

	EXPECT_EQ(m_meshShape->getNumVertices(), mesh->getNumVertices());
	EXPECT_EQ(m_meshShape->getNumEdges(), mesh->getNumEdges());
	EXPECT_EQ(m_meshShape->getNumTriangles(), mesh->getNumTriangles());
}

TEST_F(DeformableCollisionRepresentationTest, UpdateAndInitializationTest)
{
	EXPECT_ANY_THROW(m_deformableCollisionRepresentation->update(0.0));

	auto fem3DRepresentation = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("Fem3DRepresentation");
	fem3DRepresentation->setFilename(m_filename);

	// Member data 'odeState' will be created while loading.
	ASSERT_TRUE(fem3DRepresentation->initialize(m_runtime));

	// Connect Physics representation with Collision representation.
	fem3DRepresentation->setCollisionRepresentation(m_deformableCollisionRepresentation);

	// Set the shape used by Collision representation.
	m_deformableCollisionRepresentation->setShape(m_meshShape);
	EXPECT_NO_THROW(m_deformableCollisionRepresentation->initialize(m_runtime));
	EXPECT_NO_THROW(m_deformableCollisionRepresentation->wakeUp());
	EXPECT_NO_THROW(m_deformableCollisionRepresentation->update(0.0));
	EXPECT_TRUE(m_deformableCollisionRepresentation->isActive());

	// The MeshShape fails to update due to the normal calculation, making the collision rep inactive
	auto state = fem3DRepresentation->getCurrentState();
	state->getPositions().setConstant(0.0);
	fem3DRepresentation->setInitialState(state);
	EXPECT_NO_THROW(m_deformableCollisionRepresentation->update(0.0));
	EXPECT_FALSE(m_deformableCollisionRepresentation->isActive());
}

} // namespace Physics
} // namespace SurgSim
