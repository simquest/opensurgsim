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

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

namespace
{
std::string findFile(const std::string& filename)
{
	std::vector<std::string> paths;
	paths.push_back("Data/PlyReaderTests");
	SurgSim::Framework::ApplicationData data(paths);

	return data.findFile(filename);
}

typedef SurgSim::DataStructures::TriangleMeshPlain MeshType;

}

namespace SurgSim
{
namespace DataStructures
{

class PlyReaderTests : public ::testing::Test
{
};

TEST_F(PlyReaderTests, InitTest)
{
	ASSERT_NO_THROW(PlyReader("xxx"));
	ASSERT_NO_THROW(PlyReader(findFile("Cube.ply")));

	PlyReader reader(findFile("Cube.ply"));
	EXPECT_TRUE(reader.isValid());

	PlyReader reader2(findFile("xxx"));
	EXPECT_FALSE(reader2.isValid());
}

TEST_F(PlyReaderTests, FindElementsAndProperties)
{
	PlyReader reader(findFile("Cube.ply"));

	EXPECT_TRUE(reader.hasElement("vertex"));
	EXPECT_TRUE(reader.hasElement("face"));
	EXPECT_FALSE(reader.hasElement("xxx"));

	EXPECT_TRUE(reader.hasProperty("vertex", "x"));
	EXPECT_TRUE(reader.hasProperty("vertex", "y"));
	EXPECT_TRUE(reader.hasProperty("face", "nature"));
	EXPECT_TRUE(reader.hasProperty("face", "vertex_indices"));
	EXPECT_FALSE(reader.hasProperty("xxx", "vertex_indices"));
	EXPECT_FALSE(reader.hasProperty("vertex", "vertex_indices"));
}

TEST_F(PlyReaderTests, IsScalar)
{
	PlyReader reader(findFile("Testdata.ply"));

	EXPECT_TRUE(reader.isScalar("vertex", "x"));
	EXPECT_FALSE(reader.isScalar("face", "vertex_indices"));
	EXPECT_TRUE(reader.isScalar("face", "extra"));

	EXPECT_FALSE(reader.isScalar("xxx", "xxx"));
	EXPECT_FALSE(reader.isScalar("vertex", "xxx"));
	EXPECT_FALSE(reader.isScalar("face", "xxx"));
}

class TestData
{
public:
	void* beginVertices(const std::string& elementName, size_t vertices)
	{
		vertexData.overrun = 0;
		vertexInitCount = vertices;
		vertexRunningCount = 0;
		endVerticesCalled = false;
		return &vertexData;
	}

	void newVertex(const std::string& elementName)
	{
		++vertexRunningCount;
		vertices.push_back(Vector3d(vertexData.x, vertexData.y, vertexData.z));
	}

	void endVertices(const std::string& elementName)
	{
		endVerticesCalled = true;
	}

	void* beginFaces(const std::string& elementName, size_t faces)
	{
		faceInitCount = faces;
		faceRunningCount = 0;
		faceData.overrun = 0;
		faceData.faceCount = 0;
		faceData.faces = nullptr;
		return &faceData;
	}

	void newFace(const std::string& elementName)
	{
		++faceRunningCount;
		std::vector<unsigned int> face;
		for (unsigned int i = 0; i < faceData.faceCount; ++i)
		{
			face.push_back(faceData.faces[i]);
		}
		faces.push_back(face);
		extras.push_back(faceData.extra);
		natures.push_back(faceData.nature);
	}


	struct VertexData
	{
		double x;
		double y;
		double z;
		int64_t overrun;
	};

	struct FaceData
	{
		unsigned int faceCount;
		unsigned int* faces;
		int extra;
		unsigned int nature;
		int64_t overrun;
	};

	VertexData vertexData;
	size_t vertexInitCount;
	int vertexRunningCount;
	bool endVerticesCalled;

	FaceData faceData;
	size_t faceInitCount;
	int faceRunningCount;

	std::vector<Vector3d> vertices;
	std::vector<std::vector<unsigned int>> faces;
	std::vector<int> extras;
	std::vector<unsigned int> natures;
};

TEST_F(PlyReaderTests, ElementTwoPropertyTest)
{
	TestData testData;
	PlyReader reader(findFile("Cube.ply"));

	EXPECT_TRUE(reader.requestElement("face",
									  std::bind(&TestData::beginFaces, &testData,
											  std::placeholders::_1, std::placeholders::_2),
									  std::bind(&TestData::newFace, &testData, std::placeholders::_1),
									  nullptr));

	EXPECT_TRUE(reader.requestScalarProperty("face", "nature",
										   PlyReader::TYPE_UNSIGNED_INT,
										   offsetof(TestData::FaceData, nature)));

	EXPECT_TRUE(reader.requestListProperty("face", "vertex_indices",
										   PlyReader::TYPE_UNSIGNED_INT,
										   offsetof(TestData::FaceData, faces),
										   PlyReader::TYPE_UNSIGNED_INT,
										   offsetof(TestData::FaceData, faceCount)));

	ASSERT_NO_THROW(reader.parseFile());

	EXPECT_EQ(11, testData.natures[0]);
	EXPECT_EQ(0, testData.natures[8]);
}

TEST_F(PlyReaderTests, ScalarReadTest)
{
	TestData testData;
	PlyReader reader(findFile("Testdata.ply"));
	EXPECT_TRUE(reader.requestElement("vertex",
									  std::bind(&TestData::beginVertices, &testData,
											  std::placeholders::_1, std::placeholders::_2),
									  std::bind(&TestData::newVertex, &testData, std::placeholders::_1),
									  std::bind(&TestData::endVertices, &testData, std::placeholders::_1)));

	/// Should not be able to register the element twice
	EXPECT_FALSE(reader.requestElement("vertex",
									   std::bind(&TestData::beginVertices, &testData,
											   std::placeholders::_1, std::placeholders::_2),
									   std::bind(&TestData::newVertex, &testData, std::placeholders::_1),
									   std::bind(&TestData::endVertices, &testData, std::placeholders::_1)));

	EXPECT_TRUE(reader.requestScalarProperty(
					"vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(TestData::VertexData, x)));
	EXPECT_FALSE(reader.requestScalarProperty(
					 "vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(TestData::VertexData, x)));

	EXPECT_TRUE(reader.requestScalarProperty(
					"vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(TestData::VertexData, y)));
	EXPECT_TRUE(reader.requestScalarProperty(
					"vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(TestData::VertexData, z)));

	ASSERT_NO_THROW(reader.parseFile());
	EXPECT_EQ(0L, testData.vertexData.overrun);
	EXPECT_EQ(4, testData.vertexInitCount);
	EXPECT_EQ(4, testData.vertexRunningCount);

	EXPECT_EQ(4u, testData.vertices.size());
	for (size_t i = 0; i < testData.vertices.size(); ++i)
	{
		double sign = (i % 2 == 0) ? 1 : -1;
		Vector3d expected(static_cast<double>(sign * (i + 1)),
						  static_cast<double>(sign * (i + 2)),
						  static_cast<double>(sign * (i + 3)));
		EXPECT_TRUE(expected.isApprox(testData.vertices[i])) << expected << testData.vertices[i];
	}
}


TEST_F(PlyReaderTests, ListReadTest)
{
	TestData testData;
	PlyReader reader(findFile("Testdata.ply"));
	EXPECT_TRUE(reader.requestElement("face",
									  std::bind(&TestData::beginFaces, &testData,
											  std::placeholders::_1, std::placeholders::_2),
									  std::bind(&TestData::newFace, &testData, std::placeholders::_1),
									  nullptr));
	EXPECT_TRUE(reader.requestListProperty("face", "vertex_indices",
										   PlyReader::TYPE_UNSIGNED_INT,
										   offsetof(TestData::FaceData, faces),
										   PlyReader::TYPE_UNSIGNED_INT,
										   offsetof(TestData::FaceData, faceCount)));
	EXPECT_TRUE(reader.requestScalarProperty(
					"face", "extra", PlyReader::TYPE_INT, offsetof(TestData::FaceData, extra)));

	ASSERT_NO_THROW(reader.parseFile());
	EXPECT_EQ(0L, testData.faceData.overrun);
	EXPECT_EQ(4, testData.faceInitCount);
	EXPECT_EQ(4, testData.faceRunningCount);

	EXPECT_EQ(4u, testData.faces.size());
	EXPECT_EQ(4u, testData.extras.size());

	unsigned int expected = 0;
	for (size_t i = 0; i < testData.faces.size(); ++i)
	{
		std::vector<unsigned int> face = testData.faces[i];
		EXPECT_EQ(i + 1, face.size());
		EXPECT_EQ(-static_cast<int>(i), testData.extras[i]);

		for (size_t j = 0; j < face.size(); ++j)
		{
			EXPECT_EQ(expected, face[j]);
			++expected;
		}
	}
}

TEST_F(PlyReaderTests, TriangleMeshDelegateTest)
{
	PlyReader reader(findFile("Cube.ply"));
	auto delegate = std::make_shared<TriangleMeshPlyReaderDelegate<MeshType>>();

	// parseWithDeletegate() will first call setDelegate(), then parseFile() if previous step successed.
	EXPECT_NO_THROW(EXPECT_TRUE(reader.parseWithDelegate(delegate)));

	auto mesh = delegate->getMesh();
	EXPECT_EQ(26u, mesh->getNumVertices());
	EXPECT_EQ(12u, mesh->getNumTriangles());

	// The first and last vertices from the file
	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex25(-1.0, -1.0, 1.0);

	EXPECT_TRUE(vertex0.isApprox(mesh->getVertex(0).position));
	EXPECT_TRUE(vertex25.isApprox(mesh->getVertex(25).position));

	std::array<size_t, 3> triangle0 = {0, 1, 2};
	std::array<size_t, 3> triangle11 = {10, 25, 11};

	EXPECT_EQ(triangle0, mesh->getTriangle(0).verticesId);
	EXPECT_EQ(triangle11, mesh->getTriangle(11).verticesId);
}

} // DataStructures
} // SurgSim
