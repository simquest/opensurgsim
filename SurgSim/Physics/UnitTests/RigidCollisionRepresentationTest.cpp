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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"

namespace
{
const double epsilon = 1e-10;
};

using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;

struct RigidCollisionRepresentationTest : public ::testing::Test
{
	void SetUp()
	{
		m_sphereShape = std::make_shared<SurgSim::Math::SphereShape>(1.0);
		m_rigidRepresentation = std::make_shared<RigidRepresentation>("RigidRepresentation");
		m_rigidRepresentationParameters.setShapeUsedForMassInertia(m_sphereShape);

		m_rigidRepresentation->setInitialParameters(m_rigidRepresentationParameters);
	}

	std::shared_ptr<SurgSim::Math::SphereShape> m_sphereShape;
	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> m_rigidCollisionRepresentation;
	std::shared_ptr<SurgSim::Physics::RigidRepresentation> m_rigidRepresentation;
	SurgSim::Physics::RigidRepresentationParameters m_rigidRepresentationParameters;
};

TEST_F(RigidCollisionRepresentationTest, InitTest)
{
	EXPECT_NO_THROW(RigidCollisionRepresentation("TestRigidCollisionRepresentation"));

	EXPECT_NO_THROW(m_rigidCollisionRepresentation =
					std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation")
	);
}

TEST_F(RigidCollisionRepresentationTest, SetGetRigidRepresentationTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	ASSERT_NO_THROW(m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation));
	EXPECT_EQ(m_rigidRepresentation, m_rigidCollisionRepresentation->getRigidRepresentation());
}

TEST_F(RigidCollisionRepresentationTest, ShapeTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation);

	EXPECT_EQ(m_sphereShape, m_rigidCollisionRepresentation->getShape());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SPHERE, m_rigidCollisionRepresentation->getShapeType());
}

TEST_F(RigidCollisionRepresentationTest, PoseTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation);

	SurgSim::Math::Quaterniond rotation(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d translation(11.0, 12.0, 13.0);
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform(rotation, translation);
	m_rigidCollisionRepresentation->setLocalPose(pose);
	EXPECT_TRUE(pose.isApprox(m_rigidCollisionRepresentation->getPose()));
}

TEST_F(RigidCollisionRepresentationTest, SerializationTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation);

	YAML::Node node;
	ASSERT_NO_THROW(node = m_rigidCollisionRepresentation);

	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> newRigidCollisionRepresentation;
	ASSERT_NO_THROW(newRigidCollisionRepresentation =
		std::dynamic_pointer_cast<SurgSim::Physics::RigidCollisionRepresentation>
			(node.as<std::shared_ptr<SurgSim::Framework::Component>>())
		);

	// Needs serialization of RigidRepresentation to work properly.
	//EXPECT_EQ(m_rigidRepresentation, newRigidCollisionRepresentation->getRigidRepresentation());
}