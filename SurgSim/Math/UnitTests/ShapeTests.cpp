//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.

#include <gtest/gtest.h>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;

using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::CylinderShape;
using SurgSim::Math::MeshShape;
using SurgSim::Math::OctreeShape;
using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;

namespace {
	const double epsilon = 1e-10;
}

class ShapeTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_rho = 9000.0;
		m_radius = 0.01;
		m_length = 0.1;
		m_size[0] = 0.1;
		m_size[1] = 0.2;
		m_size[2] = 0.3;
	}

	void TearDown()
	{
	}

	// Mass density
	double m_rho;

	// Radius (sphere/cylinder/capsule)
	double m_radius;

	// Length (cylinder/capsule)
	double m_length;

	// Size (box)
	double m_size[3];
};

TEST_F(ShapeTest, SphereSerialize)
{
	std::shared_ptr<Shape> sphere;
	EXPECT_NO_THROW({sphere = Shape::getFactory().create("SurgSim::Math::SphereShape");});
	sphere->setValue("Radius", m_radius);

	YAML::Node node;
	ASSERT_NO_THROW({node = sphere->encode();});

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	SphereShape actualSphere;
	ASSERT_NO_THROW({actualSphere.decode(node);});
	EXPECT_EQ(m_radius, actualSphere.getValue<double>("Radius"));
	EXPECT_EQ("SurgSim::Math::SphereShape", actualSphere.getClassName());
}

TEST_F(ShapeTest, Sphere)
{
	ASSERT_NO_THROW({SphereShape sphere(m_radius);});

	SphereShape sphere(m_radius);
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SPHERE, sphere.getType());
	EXPECT_EQ(m_radius, sphere.getRadius());

	const double& r = m_radius;
	const double r2 = r * r;
	double expectedVolume = 4.0 / 3.0 * M_PI * (r2 * r);
	double expectedMass = m_rho * expectedVolume;
	double coef = 2.0 / 5.0 * expectedMass * r2;
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
					   0.0, coef, 0.0,
					   0.0, 0.0, coef;

	double volume = sphere.getVolume();
	Vector3d center = sphere.getCenter();
	Matrix33d inertia = sphere.getSecondMomentOfVolume() * m_rho;

	EXPECT_NEAR(expectedVolume, volume, epsilon);
	EXPECT_TRUE(center.isZero());
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(ShapeTest, BoxSerialize)
{
	std::shared_ptr<Shape> box;
	EXPECT_NO_THROW({box = Shape::getFactory().create("SurgSim::Math::BoxShape");});
	box->setValue("SizeX", m_size[0]);
	box->setValue("SizeY", m_size[1]);
	box->setValue("SizeZ", m_size[2]);

	YAML::Node node;
	ASSERT_NO_THROW({node = box->encode();});

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(3u, node.size());

	BoxShape actualBox;
	ASSERT_NO_THROW({actualBox.decode(node);});
	EXPECT_EQ(m_size[0], actualBox.getValue<double>("SizeX"));
	EXPECT_EQ(m_size[1], actualBox.getValue<double>("SizeY"));
	EXPECT_EQ(m_size[2], actualBox.getValue<double>("SizeZ"));
	EXPECT_EQ("SurgSim::Math::BoxShape", actualBox.getClassName());
}

TEST_F(ShapeTest, Box)
{
	ASSERT_NO_THROW({BoxShape box(m_size[0], m_size[1], m_size[2]);});

	Vector3d size(m_size[0], m_size[1], m_size[2]);
	BoxShape box(m_size[0], m_size[1], m_size[2]);
	EXPECT_EQ(m_size[0], box.getSizeX());
	EXPECT_EQ(m_size[1], box.getSizeY());
	EXPECT_EQ(m_size[2], box.getSizeZ());
	EXPECT_TRUE(box.getSize().isApprox(size));
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_BOX, box.getType());

	double expectedVolume = m_size[0] * m_size[1] * m_size[2];
	double expectedMass = m_rho * expectedVolume;
	double coef = 1.0 / 12.0 * expectedMass;
	double x2 = m_size[0] * m_size[0];
	double y2 = m_size[1] * m_size[1];
	double z2 = m_size[2] * m_size[2];
	Matrix33d expectedInertia;
	expectedInertia << coef * (y2 + z2), 0.0, 0.0,
					   0.0, coef * (x2 + z2), 0.0,
					   0.0, 0.0, coef * (x2 + y2);

	double volume = box.getVolume();
	Vector3d center = box.getCenter();
	Matrix33d inertia = box.getSecondMomentOfVolume() * m_rho;

	EXPECT_NEAR(expectedVolume, volume, epsilon);
	EXPECT_TRUE(center.isZero());
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(ShapeTest, CylinderSerialize)
{
	std::shared_ptr<Shape> cylinder;
	EXPECT_NO_THROW({cylinder = Shape::getFactory().create("SurgSim::Math::CylinderShape");});
	cylinder->setValue("Length", m_length);
	cylinder->setValue("Radius", m_radius);

	YAML::Node node;
	ASSERT_NO_THROW({node = cylinder->encode();});

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(2u, node.size());

	CylinderShape actualCylinder;
	ASSERT_NO_THROW({actualCylinder.decode(node);});
	EXPECT_EQ(m_length, actualCylinder.getValue<double>("Length"));
	EXPECT_EQ(m_radius, actualCylinder.getValue<double>("Radius"));
	EXPECT_EQ("SurgSim::Math::CylinderShape", actualCylinder.getClassName());
}

TEST_F(ShapeTest, Cylinder)
{
	ASSERT_NO_THROW({CylinderShape cyliner(m_length, m_radius);});

	CylinderShape cylinder(m_length, m_radius);
	EXPECT_EQ(m_length, cylinder.getLength());
	EXPECT_EQ(m_radius, cylinder.getRadius());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_CYLINDER, cylinder.getType());

	double expectedVolume = M_PI * m_radius * m_radius * m_length;
	double expectedMass = m_rho * expectedVolume;

	double r1sq = m_radius * m_radius;
	double l2 = m_length * m_length;
	double coefDir = 1.0 /  2.0 * expectedMass * (r1sq);
	double coef    = 1.0 / 12.0 * expectedMass * (3.0 * (r1sq) + l2);
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
					   0.0, coefDir, 0.0,
					   0.0, 0.0, coef;

	double volume = cylinder.getVolume();
	Vector3d center = cylinder.getCenter();
	Matrix33d inertia = cylinder.getSecondMomentOfVolume() * m_rho;

	EXPECT_NEAR(expectedVolume, volume, epsilon);
	EXPECT_TRUE(center.isZero());
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(ShapeTest, CapsuleSerialize)
{
	std::shared_ptr<Shape> capsule;
	EXPECT_NO_THROW({capsule = Shape::getFactory().create("SurgSim::Math::CapsuleShape");});
	capsule->setValue("Length", m_length);
	capsule->setValue("Radius", m_radius);

	YAML::Node node;
	ASSERT_NO_THROW({node = capsule->encode();});

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(2u, node.size());

	CapsuleShape actualCapsule;
	ASSERT_NO_THROW({actualCapsule.decode(node);});
	EXPECT_EQ(m_length, actualCapsule.getValue<double>("Length"));
	EXPECT_EQ(m_radius, actualCapsule.getValue<double>("Radius"));
	EXPECT_EQ("SurgSim::Math::CapsuleShape", actualCapsule.getClassName());
}

TEST_F(ShapeTest, Capsule)
{
	ASSERT_NO_THROW({CapsuleShape capsule(m_length, m_radius);});

	CapsuleShape capsule(m_length, m_radius);
	EXPECT_EQ(m_length, capsule.getLength());
	EXPECT_EQ(m_radius, capsule.getRadius());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_CAPSULE, capsule.getType());

	double r2 = m_radius * m_radius;
	double r3 = r2 * m_radius;
	double l2 = m_length * m_length;

	double volumeCylinder = M_PI * r2 * m_length;
	double massCylinder = m_rho * volumeCylinder;
	double volumeSphere = 4.0 / 3.0 * M_PI * r3;
	double massSphere = m_rho * volumeSphere;
	double expectedVolume = volumeCylinder + volumeSphere;
	double coefDir = 2.0 /  5.0 * massSphere * r2;
	double coef    = coefDir;
	coefDir += 1.0 / 2.0 * massCylinder * r2;
	coef += massSphere  * (1.0 / 4.0 * l2 + 3.0 / 8.0 * m_radius * m_length);
	coef += 1.0 / 12.0 * massCylinder * (3 * r2 + l2);
	Matrix33d expectedInertia;
	expectedInertia << coef, 0.0, 0.0,
					   0.0, coefDir, 0.0,
					   0.0, 0.0, coef;

	double volume = capsule.getVolume();
	Vector3d center = capsule.getCenter();
	Matrix33d inertia = capsule.getSecondMomentOfVolume() * m_rho;

	EXPECT_NEAR(expectedVolume, volume, epsilon);
	EXPECT_TRUE(center.isZero());
	EXPECT_TRUE(expectedInertia.isApprox(inertia));
}

TEST_F(ShapeTest, Octree)
{
	OctreeShape::NodeType::AxisAlignedBoundingBox boundingBox(Vector3d::Zero(), m_size);
	std::shared_ptr<OctreeShape::NodeType> node = std::make_shared<OctreeShape::NodeType>(boundingBox);

	{
		ASSERT_NO_THROW({OctreeShape octree;});
		ASSERT_NO_THROW({OctreeShape octree(*node);});
	}

	{
		OctreeShape octree;
		EXPECT_EQ(nullptr, octree.getRootNode());
		octree.setRootNode(node);
		EXPECT_EQ(node, octree.getRootNode());
	}

	{
		OctreeShape octree;
		octree.setRootNode(node);
		EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_OCTREE, octree.getType());
		EXPECT_THROW(octree.getVolume(), SurgSim::Framework::AssertionFailure);
		EXPECT_TRUE(octree.getCenter().isApprox(Vector3d::Zero(), epsilon));
		EXPECT_THROW(octree.getSecondMomentOfVolume(), SurgSim::Framework::AssertionFailure);
		EXPECT_EQ(octree.getClassName(), "SurgSim::Math::OctreeShape");
	}
}
