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
#include <vector>

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/MeshPlyReaderDelegate.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/MathUtilities.h"
#include "SurgSim/Testing/TestCube.h"

#include <string>

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Testing::interpolate;

namespace SurgSim
{
namespace Graphics
{

struct OsgMeshRepresentationRenderTests : public RenderTest
{

protected:
	std::vector<Vector3d> cubeVertices;
	std::vector<size_t> cubeTriangles;
	std::vector<Vector4d> cubeColors;
	std::vector<Vector2d> cubeTextures;

	std::shared_ptr<MeshRepresentation> makeRepresentation(const std::string& name)
	{
		auto meshRepresentation = std::make_shared<OsgMeshRepresentation>(name);
		meshRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.0)));
		return meshRepresentation;
	}

	std::shared_ptr<SurgSim::Graphics::Mesh> loadGraphicsMesh(const std::string& fileName)
	{
		// The PlyReader and TriangleMeshPlyReaderDelegate work together to load triangle meshes.
		SurgSim::DataStructures::PlyReader reader(fileName);
		auto delegate = std::make_shared<SurgSim::Graphics::MeshPlyReaderDelegate>();
		SURGSIM_ASSERT(reader.parseWithDelegate(delegate)) << "The input file " << fileName << " is malformed.";

		return delegate->getMesh();
	}
};

TEST_F(OsgMeshRepresentationRenderTests, StaticRotateDynamicScale)
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Scene");
	scene->addSceneElement(element);

	SurgSim::Testing::Cube::makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);

	// make a colored cube
	auto meshRepresentation1 = makeRepresentation("colormesh");
	meshRepresentation1->getMesh()->initialize(cubeVertices, cubeColors, std::vector<Vector2d>(), cubeTriangles);
	meshRepresentation1->setUpdateOptions(MeshRepresentation::UPDATE_OPTION_COLORS |
										  MeshRepresentation::UPDATE_OPTION_VERTICES);

	// make a textured cube
	auto meshRepresentation2 = makeRepresentation("textureMesh");
	meshRepresentation2->getMesh()->initialize(cubeVertices, std::vector<Vector4d>(), cubeTextures, cubeTriangles);

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
		std::pair<RigidTransform3d, RigidTransform3d> transform;
		std::pair<double, double> scale;
	};

	std::vector<InterpolationData> interpolators;
	InterpolationData interpolator;

	interpolator.transform.first =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 1.0, 1.0)), Vector3d(-0.1, 0.0, -0.2));
	interpolator.scale.first = 0.001;
	interpolator.transform.second =
		makeRigidTransform(makeRotationQuaternion(M_PI_2, Vector3d(1.0, -1.0, 1.0)), Vector3d(0.1, 0.0, -0.2));
	interpolator.scale.second = 0.03;
	interpolators.push_back(interpolator);

	interpolator.transform.first =
		makeRigidTransform(makeRotationQuaternion(-M_PI_2, Vector3d(-1.0, -1.0, 0.0)), Vector3d(0.0, -0.1, -0.2));
	interpolator.scale.first = 0.001;
	interpolator.transform.second =
		makeRigidTransform(makeRotationQuaternion(-M_PI_2, Vector3d(-1.0, 1.0 , 0.0)), Vector3d(0.0, 0.1, -0.2));
	interpolator.scale.second = 0.03;
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
			InterpolationData data = interpolators[j];

			double scale = interpolate(data.scale, t);
			RigidTransform3d transform = interpolate(data.transform, t);
			for (size_t index = 0; index < cubeVertices.size(); ++index)
			{
				newVertices[index] =  transform * (cubeVertices[index] * scale);
			}
			meshes[j]->setVertexPositions(newVertices, true);
		}

		if (i == 500)
		{
			for (size_t v = 0; v < cubeColors.size(); ++v)
			{
				//meshes[0]->getVertex(v).data.color.setValue(cubeColors[(v+numSteps)%cubeColors.size()]);
				meshes[0]->getVertex(v).data.color.setValue(Vector4d(1.0, 0.0, 0.5, 1.0));
			}
		}

		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps) * 4);
	}
}

TEST_F(OsgMeshRepresentationRenderTests, DeformableMeshRenderTest)
{

	std::string woundFilename;
	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("OsgMeshRepresentationRenderTests/wound_deformable.ply",
				&woundFilename));

	std::string textureFilename;
	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("OsgMeshRepresentationRenderTests/CheckerBoard.png",
				&textureFilename));

	// Create a triangle mesh for visualizing the surface of the finite element model
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Mesh");
	*graphics->getMesh() = SurgSim::Graphics::Mesh(*loadGraphicsMesh(woundFilename));


	// Create material to transport the Textures
	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>();
	auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
	texture->loadImage(textureFilename);
	auto diffuseMapUniform =
		std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("diffuseMap");
	diffuseMapUniform->set(texture);
	material->addUniform(diffuseMapUniform);
	graphics->setMaterial(material);

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Wound");
	sceneElement->addComponent(graphics);

	scene->addSceneElement(sceneElement);
	viewElement->setPose(SurgSim::Math::makeRigidTransform(
							 Vector3d(-0.1, 0.1, -0.1),
							 Vector3d(0.0, 0.0, 0.0),
							 Vector3d(0.0, 0.0, 1.0)));

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	runtime->stop();

}

}; // namespace Graphics
}; // namespace SurgSim
