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

#include <algorithm>
#include <memory>
#include <vector>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/OsgMeshRepresentation.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgAxesRepresentation.h>
#include <SurgSim/Graphics/OsgUniform.h>

#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/Blocks/BasicSceneElement.h>

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Testing/MathUtilities.h>

#include <SurgSim/Graphics/RenderTests/RenderTest.h>
#include <SurgSim/Graphics/RenderTests/TestCube.h>



using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

using SurgSim::Testing::interpolate;
using SurgSim::Testing::interpolatePose;

#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Behavior.h>
#include <utility>

namespace SurgSim
{
namespace Graphics
{

struct OsgMeshrepresentationRenderTests : public RenderTest
{

protected:

	std::vector<Vector3d> cubeVertices;
	std::vector<unsigned int> cubeTriangles;
	std::vector<Vector4d> cubeColors;
	std::vector<Vector2d> cubeTextures;

	std::shared_ptr<Mesh> makeMesh(const std::vector<Vector3d>& vertices,
								   const std::vector<Vector4d>& colors,
								   const std::vector<Vector2d>& textures,
								   const std::vector<unsigned int>& triangles)
	{
		std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
		size_t i = 0;
		for (auto it = std::begin(vertices); it != std::end(vertices); ++it)
		{
			VertexData data;
			if (! colors.empty())
			{
				data.color.setValue(colors[i]);
			}
			if (! textures.empty())
			{
				data.texture.setValue(textures[i]);
			}
			mesh->addVertex(Mesh::VertexType(*it, data));
			++i;
		}

		for (size_t i = 0; i < triangles.size()/3; ++i )
		{
			Mesh::TriangleType::IdType ids = {{triangles[3*i], triangles[3*i+1],triangles[3*i+2]}};
			Mesh::TriangleType triangle(ids, SurgSim::Graphics::TriangleData());
			mesh->addTriangle(triangle);
		}

		return mesh;
	}

	std::shared_ptr<MeshRepresentation> makeRepresentation(std::shared_ptr<Mesh> mesh, const std::string& name)
	{
		auto meshRepresentation = std::make_shared<OsgMeshRepresentation>(name);

		EXPECT_TRUE(mesh->isValid());

		meshRepresentation->setMesh(mesh);
		meshRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0,0.0,0.0)));

		//meshRepresentation->setDrawAsWireFrame(true);

		return meshRepresentation;
	}
};

TEST_F(OsgMeshrepresentationRenderTests, StaticRotateDynamicScale)
{
	std::shared_ptr<SurgSim::Blocks::BasicSceneElement> element =
		std::make_shared<SurgSim::Blocks::BasicSceneElement>("Scene");
	scene->addSceneElement(element);

	SurgSim::Testing::Cube::makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);

	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> meshRepresentation1;
	meshRepresentation1 = makeRepresentation(
		makeMesh(cubeVertices, cubeColors, std::vector<Vector2d>(), cubeTriangles), "colormesh");
	meshRepresentation1->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0,0.0,0.0)));

	// make a textured cube
	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> meshRepresentation2;
	meshRepresentation2 = makeRepresentation(makeMesh
		(cubeVertices, std::vector<Vector4d>(), cubeTextures, cubeTriangles), "texturemesh");
	meshRepresentation2->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(-0.0,0.0,-0.0)));
	auto material = std::make_shared<OsgMaterial>();
	auto texture = std::make_shared<OsgTexture2d>();
	texture->loadImage(applicationData->findFile("OsgMeshRepresentationRenderTests/cube.png"));

	auto uniform2d = std::make_shared<OsgUniform<std::shared_ptr<SurgSim::Graphics::OsgTexture2d>>>("oss_diffuseMap");
	uniform2d->set(texture);
	material->addUniform(uniform2d);
	meshRepresentation2->setMaterial(material);

	element->addComponent(meshRepresentation1);
	element->addComponent(meshRepresentation2);

	auto axes = std::make_shared<OsgAxesRepresentation>("Origin");
	viewElement->addComponent(axes);

	struct InterpolationData
	{
	public:
		RigidTransform3d transform;
		double scale;
	};

	typedef std::pair<InterpolationData, InterpolationData> Interpolator;

	std::vector<Interpolator> interpolators;
	Interpolator interpolator;

	interpolator.first.transform =
		makeRigidTransform(makeRotationQuaternion(0.0,Vector3d(1.0,1.0,1.0)), Vector3d(-0.1, 0.0, -0.2));
	interpolator.first.scale = 0.001;
	interpolator.second.transform =
		makeRigidTransform(makeRotationQuaternion(M_PI_2,Vector3d(1.0,-1.0,1.0)), Vector3d(0.1, 0.0, -0.2));
	interpolator.second.scale = 0.03;
	interpolators.push_back(interpolator);

	interpolator.first.transform =
		makeRigidTransform(makeRotationQuaternion(-M_PI_2,Vector3d(-1.0,-1.0,0.0)), Vector3d(0.0, -0.1, -0.2));
	interpolator.first.scale = 0.001;
	interpolator.second.transform =
		makeRigidTransform(makeRotationQuaternion(-M_PI_2,Vector3d(-1.0,1.0,0.0)), Vector3d(0.0, 0.1, -0.2));
	interpolator.second.scale = 0.03;
	interpolators.push_back(interpolator);

	std::vector<std::shared_ptr<Mesh>> meshes;
	meshes.push_back(meshRepresentation1->getMesh());
	meshes.push_back(meshRepresentation2->getMesh());

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());

	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	int numSteps = 1000;

	std::vector<Vector3d> newVertices(cubeVertices.size());

	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;

		for (size_t j = 0; j < interpolators.size(); ++j)
		{
			InterpolationData from = interpolators[j].first;
			InterpolationData to = interpolators[j].second;

			double scale = interpolate(from.scale, to.scale, t);
			RigidTransform3d transform = interpolate(from.transform, to.transform, t);
			for (size_t index = 0; index < cubeVertices.size(); ++index)
			{
				newVertices[index] =  transform * (cubeVertices[index] * scale);
			}
			meshes[j]->setVertexPositions(newVertices,true);
		}

		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps)*4);
	}
}

}; // namespace Graphics
}; // namespace SurgSim
