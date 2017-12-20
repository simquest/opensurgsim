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

#include <vector>
#include <gtest/gtest.h>
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Testing/TestCube.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::UnalignedVector4d;

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Graphics
{

struct MeshTests : public ::testing::Test
{
public:

	virtual void SetUp()
	{
		SurgSim::Testing::Cube::makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);
	}


	std::vector<Vector3d> cubeVertices;
	std::vector<size_t> cubeTriangles;
	std::vector<Vector4d> cubeColors;
	std::vector<Vector2d> cubeTextures;
};


TEST_F(MeshTests, MakeMeshTestWorking)
{
	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	ASSERT_NO_THROW(
	{
		mesh->initialize(cubeVertices, cubeColors, cubeTextures, cubeTriangles);
	}
	);

	EXPECT_TRUE(mesh->isValid());

	EXPECT_EQ(cubeVertices.size(), mesh->getNumVertices());
	EXPECT_EQ(cubeTriangles.size() / 3, mesh->getNumTriangles());

	for (size_t i = 0; i < cubeVertices.size(); ++i)
	{
		EXPECT_TRUE(cubeColors[i] == mesh->getVertex(i).data.color.getValue());
		EXPECT_TRUE(cubeTextures[i] == mesh->getVertex(i).data.texture.getValue());
	}
}

TEST_F(MeshTests, MakeMeshColors)
{
	std::vector<Vector4d> emptyCubeColors;
	std::vector<Vector4d> tooFewCubeColors;
	std::vector<Vector4d> tooManyCubeColors(cubeColors);
	tooFewCubeColors.push_back(Vector4d(0.0, 0.0, 0.0, 0.0));
	tooManyCubeColors.push_back(Vector4d(0.0, 0.0, 0.0, 0.0));

	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	ASSERT_NO_THROW(
	{
		mesh->initialize(cubeVertices, emptyCubeColors, cubeTextures, cubeTriangles);
	}
	);

	for (size_t i = 0; i < cubeVertices.size(); ++i)
	{
		EXPECT_FALSE(mesh->getVertex(i).data.color.hasValue());
	}


	mesh = std::make_shared<Mesh>();
	ASSERT_ANY_THROW(
	{
		mesh->initialize(cubeVertices, tooFewCubeColors, cubeTextures, cubeTriangles);
	}
	);

	mesh = std::make_shared<Mesh>();
	ASSERT_NO_THROW(
	{
		mesh->initialize(cubeVertices, tooManyCubeColors, cubeTextures, cubeTriangles);
	}
	);

}

TEST_F(MeshTests, MakeMeshTextures)
{
	std::vector<Vector2d> emptyCubeTextures;
	std::vector<Vector2d> tooFewCubeTextures;
	std::vector<Vector2d> tooManyCubeTextures(cubeTextures);
	tooFewCubeTextures.push_back(Vector2d(0.0, 0.0));
	tooManyCubeTextures.push_back(Vector2d(0.0, 0.0));

	std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
	ASSERT_NO_THROW(
	{
		mesh->initialize(cubeVertices, cubeColors, emptyCubeTextures, cubeTriangles);
	}
	);

	for (size_t i = 0; i < cubeVertices.size(); ++i)
	{
		EXPECT_FALSE(mesh->getVertex(i).data.texture.hasValue());
	}

	mesh = std::make_shared<Mesh>();
	ASSERT_ANY_THROW(
	{
		mesh->initialize(cubeVertices, cubeColors, tooFewCubeTextures, cubeTriangles);
	}
	);

	mesh = std::make_shared<Mesh>();
	ASSERT_NO_THROW(
	{
		mesh->initialize(cubeVertices, cubeColors, tooManyCubeTextures, cubeTriangles);
	}
	);

}


}; // namespace Graphics
}; // namespace SurgSim
