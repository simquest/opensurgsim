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
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/MeshPlyReaderDelegate.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgProgram.h"
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
using SurgSim::Math::Vector4f;
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
};

TEST_F(OsgMeshRepresentationRenderTests, BasicCubeTest)
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Scene");
	scene->addSceneElement(element);

	// make an empty mesh
	auto meshRepresentation0 = makeRepresentation("emptymesh");
	meshRepresentation0->getMesh()->initialize(cubeVertices, cubeColors, std::vector<Vector2d>(), cubeTriangles);
	meshRepresentation0->setUpdateOptions(MeshRepresentation::UPDATE_OPTION_COLORS |
										  MeshRepresentation::UPDATE_OPTION_VERTICES);

	SurgSim::Testing::Cube::makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);

	// make a colored cube
	auto meshRepresentation1 = makeRepresentation("colormesh");
	meshRepresentation1->getMesh()->initialize(cubeVertices, cubeColors, std::vector<Vector2d>(), cubeTriangles);
	meshRepresentation1->setUpdateOptions(MeshRepresentation::UPDATE_OPTION_COLORS |
										  MeshRepresentation::UPDATE_OPTION_VERTICES);

	// make a textured cube
	auto meshRepresentation2 = makeRepresentation("textureMesh");
	meshRepresentation2->getMesh()->initialize(cubeVertices, std::vector<Vector4d>(), cubeTextures, cubeTriangles);

	auto material = std::make_shared<OsgMaterial>("material");
	auto texture = std::make_shared<OsgTexture2d>();
	texture->loadImage(applicationData->findFile("Textures/CubeMap_rgb_rotate.png"));

	auto uniform2d = std::make_shared<OsgUniform<std::shared_ptr<SurgSim::Graphics::OsgTexture2d>>>("oss_diffuseMap");
	uniform2d->set(texture);
	material->addUniform(uniform2d);
	meshRepresentation2->setMaterial(material);

	element->addComponent(material);
	element->addComponent(meshRepresentation0);
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
	meshes.push_back(std::make_shared<Graphics::Mesh>(*meshRepresentation2->getMesh()));

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());

	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

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
			if (j == 0)
			{
				// Not threadsafe update
				meshes[0]->dirty();
			}
			else
			{
				// threadsafe update
				meshRepresentation2->updateMesh(*meshes[1]);
			}
		}

		if (i == 500)
		{
			for (size_t v = 0; v < cubeColors.size(); ++v)
			{
				meshes[0]->getVertex(v).data.color.setValue(Vector4d(1.0, 0.0, 0.5, 1.0));
			}
		}

		/// The total number of steps should complete in 4 seconds
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000 / numSteps));
	}
}

TEST_F(OsgMeshRepresentationRenderTests, TextureTest)
{
	// Create a triangle mesh for visualizing the surface of the finite element model
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Mesh");
	graphics->loadMesh("Geometry/wound_deformable_with_texture.ply");

	// Create material to transport the Textures
	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>("material");
	auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();

	std::string textureFilename;
	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/checkered.png", &textureFilename));
	texture->loadImage(textureFilename);
	auto diffuseMapUniform =
		std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("diffuseMap");
	diffuseMapUniform->set(texture);
	material->addUniform(diffuseMapUniform);
	graphics->setMaterial(material);

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Wound");
	sceneElement->addComponent(graphics);
	sceneElement->addComponent(material);

	scene->addSceneElement(sceneElement);
	viewElement->setPose(SurgSim::Math::makeRigidTransform(
							 Vector3d(-0.1, 0.1, -0.1),
							 Vector3d(0.0, 0.0, 0.0),
							 Vector3d(0.0, 0.0, 1.0)));

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	runtime->stop();

}

TEST_F(OsgMeshRepresentationRenderTests, ShaderTest)
{
	std::string textureFilename;
	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/wound_deformable.png",
				&textureFilename));

	// Create a triangle mesh for visualizing the surface of the finite element model
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Mesh");
	graphics->loadMesh("Geometry/wound_deformable_with_texture.ply");

	// Create material to transport the Textures

	auto material = std::make_shared<OsgMaterial>("material");
	auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/ds_mapping_material");
	ASSERT_TRUE(program != nullptr);
	material->setProgram(program);
	{
		auto uniform = std::make_shared<SurgSim::Graphics::OsgUniform<Vector4f>>("diffuseColor");
		material->addUniform(uniform);
		material->setValue("diffuseColor", SurgSim::Math::Vector4f(0.8, 0.8, 0.8, 1.0));
	}

	{
		auto uniform = std::make_shared<SurgSim::Graphics::OsgUniform<Vector4f>>("specularColor");
		material->addUniform(uniform);
		material->setValue("specularColor", SurgSim::Math::Vector4f(0.01, 0.01, 0.01, 1.0));
	}

	{
		auto uniform = std::make_shared<SurgSim::Graphics::OsgUniform<float>>("shininess");
		material->addUniform(uniform);
		material->setValue("shininess", 32.0f);
	}

	{
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		texture->loadImage(textureFilename);
		auto uniform =
			std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("diffuseMap");
		uniform->set(texture);
		material->addUniform(uniform);
	}

	{
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		std::string blackTexture;
		ASSERT_TRUE(applicationData->tryFindFile("Textures/black.png", &blackTexture));

		texture->loadImage(blackTexture);
		auto uniform =
			std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("shadowMap");
		uniform->set(texture);
		uniform->setMinimumTextureUnit(8);
		material->addUniform(uniform);
	}


	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Wound");
	sceneElement->addComponent(graphics);

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	scene->addSceneElement(sceneElement);
	viewElement->setPose(SurgSim::Math::makeRigidTransform(
							 Vector3d(-0.1, 0.1, -0.1),
							 Vector3d(0.0, 0.0, 0.0),
							 Vector3d(0.0, 0.0, 1.0)));

	viewElement->getCamera()->setAmbientColor(Vector4d(0.2, 0.2, 0.2, 1.0));
	viewElement->getCamera()->setMaterial(material);
	viewElement->addComponent(material);
	viewElement->addComponent(light);

	std::dynamic_pointer_cast<SurgSim::Graphics::OsgView>(viewElement->getView())->setOsgMapsUniforms(true);

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	runtime->stop();

}

TEST_F(OsgMeshRepresentationRenderTests, TriangleDeletionTest)
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Scene");
	scene->addSceneElement(element);

	SurgSim::Testing::Cube::makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);

	// make a colored cube
	auto meshRepresentation1 = makeRepresentation("colormesh");
	meshRepresentation1->getMesh()->initialize(cubeVertices, cubeColors, std::vector<Vector2d>(), cubeTriangles);
	meshRepresentation1->setUpdateOptions(MeshRepresentation::UPDATE_OPTION_COLORS |
										  MeshRepresentation::UPDATE_OPTION_VERTICES |
										  MeshRepresentation::UPDATE_OPTION_TRIANGLES);

	element->addComponent(meshRepresentation1);

	auto axes = std::make_shared<OsgAxesRepresentation>("Origin");
	viewElement->addComponent(axes);
	viewElement->setPose(SurgSim::Math::makeRigidTransform(
							 Vector3d(-2.0, -2.0, -2.0),
							 Vector3d(0.0, 0.0, 0.0),
							 Vector3d(0.0, 0.0, 1.0)));
// 	auto view = std::dynamic_pointer_cast<SurgSim::Graphics::OsgView>(viewElement->getView());
// 	view->enableManipulator(true);

	struct InterpolationData
	{
	public:
		std::pair<RigidTransform3d, RigidTransform3d> transform;
		std::pair<double, double> scale;
	};

	InterpolationData interpolator;
	interpolator.transform.first =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 1.0, 1.0)), Vector3d(-0.1, 0.0, -0.2));
	interpolator.scale.first = 0.03;
	interpolator.transform.second =
		makeRigidTransform(makeRotationQuaternion(M_PI_2 * 2, Vector3d(1.0, -1.0, 1.0)), Vector3d(0.1, 0.0, -0.2));
	interpolator.scale.second = 0.03;

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());

	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	int numSteps = 1000;

	std::vector<Vector3d> newVertices(cubeVertices.size());

	size_t triangleCount = meshRepresentation1->getMesh()->getNumTriangles();

	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;

		if (i % (numSteps / triangleCount) == 0)
		{
			size_t triangle = meshRepresentation1->getMesh()->getNumTriangles() - 1;

			// Leave one triangle for the mesh
			if (triangle > 0)
			{
				meshRepresentation1->getMesh()->removeTriangle(triangle);
				meshRepresentation1->getMesh()->dirty();
			}
		}

		RigidTransform3d transform = interpolate(interpolator.transform, t);
		element->setPose(transform);

		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps) * 4);
	}
}

}; // namespace Graphics
}; // namespace SurgSim
