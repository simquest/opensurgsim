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

namespace SurgSim
{
namespace Graphics
{


class PoseInterpolator : public SurgSim::Framework::Behavior
{
public:
	explicit PoseInterpolator(const std::string& name) :
		Behavior(name),
		m_from(RigidTransform3d::Identity()),
		m_to(RigidTransform3d::Identity()),
		m_duration(1.0),
		m_currentTime(0.0)
	{}

	void setFrom(const SurgSim::Math::RigidTransform3d transform) {m_from.setValue(transform);}
	void setTo(const SurgSim::Math::RigidTransform3d transform) {m_to = transform;}
	void setTarget(std::shared_ptr<SurgSim::Graphics::Representation> target) {m_target = target;}

	void setDuration(double t) {m_duration = t;}

	bool doWakeUp() override
	{
		return true;
	}

	bool doInitialize() override
	{
		if (! m_from.hasValue())
		{
			m_from.setValue(m_target->getPose());
		}

		return true;
	}

	void update(double dt) override
	{
		bool result = true;
		m_currentTime += dt;

		m_target->setPose(interpolate(m_from.getValue(), m_to, m_currentTime/m_duration));

		if (m_currentTime > m_duration)
		{
			getSceneElement()->removeComponent(getName());
		}
	}

private:
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::RigidTransform3d> m_from;
	SurgSim::Math::RigidTransform3d m_to;
	std::shared_ptr<SurgSim::Graphics::Representation> m_target;

	double m_duration;
	double m_currentTime;
};


struct OsgMeshrepresentationRenderTests : public RenderTest
{

protected:

	std::vector<Vector3d> cubeVertices;
	std::vector<unsigned int> cubeTriangles;
	std::vector<Vector4d> cubeColors;
	std::vector<Vector2d> cubeTextures;

	void makeCube(std::vector<Vector3d>* vertices,
				  std::vector<Vector4d>* colors,
				  std::vector<Vector2d>* textures,
				  std::vector<unsigned int>* triangles)
	{
		using SurgSim::Testing::Cube::numVertices;
		using SurgSim::Testing::Cube::numTriangles;
		using SurgSim::Testing::Cube::vertexData;
		using SurgSim::Testing::Cube::colorData;
		using SurgSim::Testing::Cube::textureData;
		using SurgSim::Testing::Cube::triangleData;

		vertices->resize(numVertices);
		colors->resize(numVertices);
		textures->resize(numVertices);

		double scale = 0.1;

		for (int i=0; i<numVertices; ++i)
		{
			(*vertices)[i] = Vector3d(vertexData[3*i]*scale, vertexData[3*i+1]*scale, vertexData[3*i+2]*scale);
			(*colors)[i] = Vector4d(colorData[4*i], colorData[4*i+1], colorData[4*i+2], colorData[4*i+3]);
			(*textures)[i] = Vector2d(textureData[2*i], textureData[2*i+1]);
		}

		triangles->resize(numTriangles*3);
		std::copy(triangleData, triangleData+12*3,std::begin(*triangles));
	}

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
			Mesh::TriangleType::IdType ids = {triangles[3*i], triangles[3*i+1],triangles[3*i+2]};
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

TEST_F(OsgMeshrepresentationRenderTests, StaticRotate)
{
	std::shared_ptr<SurgSim::Blocks::BasicSceneElement> element =
		std::make_shared<SurgSim::Blocks::BasicSceneElement>("Scene");
	scene->addSceneElement(element);


	makeCube(&cubeVertices, &cubeColors, &cubeTextures, &cubeTriangles);
	std::shared_ptr<SurgSim::Graphics::Representation> mesh1;
	mesh1 = makeRepresentation(
		makeMesh(cubeVertices, cubeColors, std::vector<Vector2d>(), cubeTriangles), "colormesh");
	mesh1->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.15,0.0,0.0)));

	// make a textured cube
	std::shared_ptr<SurgSim::Graphics::Representation> mesh2;
	mesh2 = makeRepresentation(makeMesh
		(cubeVertices, std::vector<Vector4d>(), cubeTextures, cubeTriangles), "texturemesh");
	mesh2->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(-0.15,0.0,0.0)));
	auto material = std::make_shared<OsgMaterial>();
	auto texture = std::make_shared<OsgTexture2d>();
	texture->loadImage(applicationData->findFile("OsgMeshRepresentationRenderTests/cube.png"));

	auto uniform2d = std::make_shared<OsgUniform<std::shared_ptr<SurgSim::Graphics::OsgTexture2d>>>("oss_diffuseMap");
	uniform2d->set(texture);
	material->addUniform(uniform2d);
	mesh2->setMaterial(material);

	element->addComponent(mesh1);
	element->addComponent(mesh2);
	viewElement->enableManipulator(true);

	auto axes = std::make_shared<OsgAxesRepresentation>("Origin");
	viewElement->addComponent(axes);

	auto interpolator = std::make_shared<PoseInterpolator>("Interpolator");
	interpolator->setTarget(mesh2);
	interpolator->setDuration(2.0);
	Quaterniond quat = makeRotationQuaternion<double>(M_PI_2,Vector3d(0.0,1.0,0.0));
	interpolator->setTo(makeRigidTransform(quat,Vector3d(-0.15,0.0,0.0)));
	viewElement->addComponent(interpolator);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numSteps = 1000;

	Vector3d startAngles(0.0,0.0,0.0);
	Vector3d endAngles(M_PI_4, M_PI_2, M_PI_2);
	Vector3d startPosition (-0.1, 0.0, -0.0);
	Vector3d endPosition(0.1, 0.0, -0.4);

	graphicsManager->dumpDebugInfo();

	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

}

}; // namespace Graphics
}; // namespace SurgSim
