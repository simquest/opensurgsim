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

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/NormalData.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Shapes.h"


namespace
{
const double epsilon = 1e-10;
}

namespace SurgSim
{
namespace Math
{

TEST(ParticlesShapeTests, CanConstruct)
{
	ASSERT_NO_THROW(ParticlesShape particles());
	ASSERT_NO_THROW(ParticlesShape particles(0.01));

	ASSERT_NO_THROW(ParticlesShape particles(DataStructures::Vertices<DataStructures::NormalData>()));
	ASSERT_NO_THROW(ParticlesShape particles(DataStructures::Vertices<DataStructures::EmptyData>()));
}

TEST(ParticlesShapeTests, DefaultProperties)
{
	ParticlesShape particles;
	EXPECT_NEAR(0.0, particles.getRadius(), epsilon);
	EXPECT_EQ(SHAPE_TYPE_PARTICLES, particles.getType());
}

TEST(ParticlesShapeTests, SetGetRadius)
{
	ParticlesShape particles;
	particles.setRadius(0.1);
	EXPECT_NEAR(0.1, particles.getRadius(), epsilon);
}

TEST(ParticlesShapeTests, CopyVertices)
{
	DataStructures::Vertices<DataStructures::NormalData> otherVertices;
	otherVertices.addVertex(DataStructures::Vertex<DataStructures::NormalData>(Vector3d::Constant(3.0)));

	ParticlesShape particles(otherVertices);
	EXPECT_EQ(1, particles.getNumVertices());
	EXPECT_TRUE(particles.getVertex(0).position.isApprox(Vector3d::Constant(3.0)));
}

TEST(ParticlesShapeTests, GeometricProperties)
{
	SphereShape unitSphere(1.0);
	ParticlesShape particles(1.0);

	particles.addVertex(ParticlesShape::VertexType(Vector3d::Zero()));
	particles.update();
	EXPECT_TRUE(particles.getCenter().isApprox(Vector3d::Zero()));
	EXPECT_NEAR(unitSphere.getVolume(), particles.getVolume(), epsilon);
	EXPECT_TRUE(particles.getSecondMomentOfVolume().isApprox(unitSphere.getSecondMomentOfVolume()))
		<< unitSphere.getSecondMomentOfVolume() << std::endl << particles.getSecondMomentOfVolume();

	particles.addVertex(ParticlesShape::VertexType(Vector3d::Ones()));
	particles.update();
	EXPECT_TRUE(particles.getCenter().isApprox(Vector3d::Ones() * 0.5));
	EXPECT_NEAR(2.0 * unitSphere.getVolume(), particles.getVolume(), epsilon);
	Matrix33d distanceSkew = makeSkewSymmetricMatrix(Vector3d::Ones().eval());
	EXPECT_TRUE(particles.getSecondMomentOfVolume().isApprox(2.0 * unitSphere.getSecondMomentOfVolume()
		- unitSphere.getVolume() * distanceSkew * distanceSkew));

	particles.addVertex(ParticlesShape::VertexType(-Vector3d::Ones()));
	particles.update();
	EXPECT_TRUE(particles.getCenter().isApprox(Vector3d::Zero()));
	EXPECT_NEAR(3.0 * unitSphere.getVolume(), particles.getVolume(), epsilon);
	EXPECT_TRUE(particles.getSecondMomentOfVolume().isApprox(3 * unitSphere.getSecondMomentOfVolume()
		- 2.0 * unitSphere.getVolume() * distanceSkew * distanceSkew));
}

TEST(ParticlesShapeTests, Serialization)
{
	const double radius = 0.2;
	{
		YAML::Node node;
		node["SurgSim::Math::ParticlesShape"]["Radius"] = radius;

		std::shared_ptr<Shape> shape;
		ASSERT_NO_THROW(shape = node.as<std::shared_ptr<Shape>>());

		EXPECT_EQ("SurgSim::Math::ParticlesShape", shape->getClassName());
		EXPECT_NEAR(radius, shape->getValue<double>("Radius"), epsilon);
		EXPECT_TRUE(shape->isValid());
	}

	{
		std::shared_ptr<Shape> shape;
		ASSERT_NO_THROW(shape = Shape::getFactory().create("SurgSim::Math::ParticlesShape"));
		shape->setValue("Radius", radius);
		EXPECT_TRUE(shape->isValid());

		YAML::Node node;
		ASSERT_NO_THROW(node = shape);
		EXPECT_TRUE(node.IsMap());
		EXPECT_EQ(1u, node.size());

		ASSERT_TRUE(node["SurgSim::Math::ParticlesShape"].IsDefined());
		auto data = node["SurgSim::Math::ParticlesShape"];
		EXPECT_EQ(1u, data.size());

		std::shared_ptr<ParticlesShape> particlesShape;
		ASSERT_NO_THROW(particlesShape = std::dynamic_pointer_cast<ParticlesShape>(node.as<std::shared_ptr<Shape>>()));
		EXPECT_EQ("SurgSim::Math::ParticlesShape", particlesShape->getClassName());
		EXPECT_NEAR(radius, particlesShape->getValue<double>("Radius"), epsilon);
		EXPECT_TRUE(particlesShape->isValid());
	}
}


};
};

