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

#include <vector>

#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Testing/TestCube.h"
#include "SurgSim/Framework/Runtime.h"

#include "SurgSim/Math/Vector.h"

#include <osg/ref_ptr>
#include <osg/Geometry>
#include <osg/Array>

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

using SurgSim::Framework::Runtime;

namespace
{
	std::vector<Vector3d> cubeVertices;
	std::vector<unsigned int> cubeTriangles;
	std::vector<Vector4d> cubeColors;
	std::vector<Vector2d> cubeTextures;

}



namespace SurgSim
{
namespace Graphics
{

TEST(OsgMeshRepresentationTests, InitTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<MeshRepresentation> meshRepresentation;
	ASSERT_NO_THROW({meshRepresentation = std::make_shared<OsgMeshRepresentation>("TestMesh");});

	SurgSim::Testing::Cube::makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);

	ASSERT_NE(nullptr, meshRepresentation->getMesh());
	EXPECT_EQ(OsgMeshRepresentation::UPDATE_OPTION_VERTICES, meshRepresentation->getUpdateOptions());
};

TEST(OsgMeshRepresentationTests, InitialisationTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	auto meshRepresentation = std::make_shared<OsgMeshRepresentation>("TestMesh");

	std::shared_ptr<Mesh> mesh = meshRepresentation->getMesh();
	mesh->initialize(cubeVertices, cubeColors, cubeTextures, cubeTriangles);

	EXPECT_TRUE(meshRepresentation->initialize(runtime));
	EXPECT_TRUE(meshRepresentation->wakeUp());

	ASSERT_NO_THROW(meshRepresentation->update(0.1));

	osg::ref_ptr<osg::Geometry> geometry = meshRepresentation->getOsgGeometry();
	EXPECT_NE(nullptr, geometry);

	osg::ref_ptr<osg::Array> array = geometry->getVertexArray();
	ASSERT_NE(nullptr, array);
	EXPECT_EQ(cubeVertices.size(), array->getNumElements());

	array = geometry->getColorArray();
	ASSERT_NE(nullptr, array);
	EXPECT_EQ(cubeColors.size(), array->getNumElements());

	array = geometry->getTexCoordArray(0);
	ASSERT_NE(nullptr, array);
	EXPECT_EQ(cubeTextures.size(), array->getNumElements());

	osg::ref_ptr<osg::PrimitiveSet> primitiveSet = geometry->getPrimitiveSet(0);
	ASSERT_NE(nullptr, primitiveSet);
	EXPECT_EQ(cubeTriangles.size(), primitiveSet->getNumIndices());

}

}; // namespace Graphics
}; // namespace SurgSim
