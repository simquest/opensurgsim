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

#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"

#include "SurgSim/Graphics/Light.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderPass.h"

#include "SurgSim/Blocks/BasicSceneElement.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Matrix44f;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;

using SurgSim::Framework::Logger;

using SurgSim::Graphics::OsgUniform;
using SurgSim::Graphics::OsgTextureUniform;
using SurgSim::Graphics::OsgTexture2d;

#include <osg/Matrix>
#include <osg/Camera>

#include "Surgsim/Blocks/PoseInterpolator.h"

/// \file
/// This example creates a simple graphics scene and use the RenderPass object to show
/// a simple shadowing algorithm. For the algorithm see http://en.wikipedia.org/wiki/Shadow_mapping
/// There are two preprocessing passes that use specific shaders, plus a main pass that renders the
/// objects using a shaders that can process the output from the preprocessing steps.
/// Each shaders output becomes the input for the next shader, some parameters from other scene elements
/// is passed into the shaders as uniforms.
/// All of the information is kept up to date using a \sa TransferPropertiesBehavior
/// Both the light and the main camera are being moved through a \sa PoseInterpolator to demonstrate
/// dynamic changes and how to handle them in the rendering pipeline

namespace
{

std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>> materials;

std::shared_ptr<SurgSim::Graphics::OsgMaterial> loadMaterial(
	const SurgSim::Framework::ApplicationData& data,
	const std::string& name)
{
	std::string vertexShaderName = name+".vert";
	std::string fragmentShaderName = name+".frag";

	std::string filename;

	auto shader(std::make_shared<SurgSim::Graphics::OsgShader>());
	bool success = true;
	filename = data.findFile(vertexShaderName);
	if (filename == "")
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not find vertex shader " << vertexShaderName;
		success = false;
	}
	else if (! shader->loadVertexShaderSource(filename))
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not load vertex shader " << vertexShaderName;
		success = false;
	}


	filename = data.findFile(fragmentShaderName);
	if (filename == "")
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not find fragment shader " << fragmentShaderName;
		success = false;
	}
	if (! shader->loadFragmentShaderSource(filename))
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not load fragment shader " << fragmentShaderName;
		success = false;
	}

	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material;
	if (success)
	{
		material = std::make_shared<SurgSim::Graphics::OsgMaterial>();
		material->setShader(shader);
	}

	return material;

}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Main Light");
	light->setAmbientColor(Vector4d(0.5,0.5,0.5,1.0));
	light->setDiffuseColor(Vector4d(0.5,0.5,0.5,1.0));
	light->setSpecularColor(Vector4d(0.8,0.8,0.8,1.0));
	light->setInitialPose(makeRigidTransform(Vector3d(-4.0, 2.0, -4.0), Vector3d(0.0,0.0,0.0), Vector3d(0.0,1.0,0.0)));

	viewElement->addComponent(light);

	// Move the light from left to right over along the scene
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(Vector3d(4.0, 3.0, -4.0), Vector3d(0.0,0.0,0.0), Vector3d(0.0,1.0,0.0));
	RigidTransform3d to = makeRigidTransform(Vector3d(-4.0, 3.0, -4.0), Vector3d(0.0,0.0,0.0), Vector3d(0.0,1.0,0.0));
	interpolator->setTarget(light);
	interpolator->setStartingPose(from);
	interpolator->setDuration(10.0);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	viewElement->addComponent(interpolator);

	return viewElement;
}

/// Create the pass that renders the scene from the view of the light source
/// the identifier 'shadowing' is used in all graphic objects to mark them as used
/// in this pass
std::shared_ptr<SurgSim::Graphics::RenderPass> createLightMapPass()
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>("shadowing");
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);
	materials["depthMap"]->getShader()->setGlobalScope(true);
	pass->setMaterial(materials["depthMap"]);

	return pass;
}

/// Create the pass that renders shadowed pixels into the scene,
/// the identifier 'shadowed' can be used in all graphics objects to mark them
/// as used in this pass
std::shared_ptr<SurgSim::Graphics::RenderPass> createShadowMapPass()
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>("shadowed");
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);
	materials["shadowMap"]->getShader()->setGlobalScope(true);
	pass->setMaterial(materials["shadowMap"]);
	return pass;
}

/// A simple box as a scenelement
class SimpleBox : public SurgSim::Blocks::BasicSceneElement
{
public:
	explicit SimpleBox(const std::string& name) : BasicSceneElement(name)
	{
		m_box = std::make_shared<SurgSim::Graphics::OsgBoxRepresentation>(getName()+" Graphics");
		m_box->setInitialPose(RigidTransform3d::Identity());
		
		// The material that this object uses
		m_box->setMaterial(materials["basicShadowed"]);

		// Assign this to the pass for shadowing objects
		m_box->addGroupReference("shadowing");

		// Assign this to the pass for shadowed objects
		m_box->addGroupReference("shadowed");
	}

	virtual bool doInitialize() override
	{
		addComponent(m_box);
		return true;
	}

	void setSize(double width, double height, double length)
	{
		m_box->setSize(width, height, length);
	}

	void setPose(const RigidTransform3d& pose)
	{
		m_box->setPose(pose);
	}

private:
	std::shared_ptr<SurgSim::Graphics::OsgBoxRepresentation> m_box;
};


class SimpleSphere : public SurgSim::Blocks::BasicSceneElement
{
public:
	explicit SimpleSphere(const std::string& name) : BasicSceneElement(name)
	{
		m_sphere = std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>(getName()+" Graphics");
		m_sphere->setInitialPose(RigidTransform3d::Identity());
		m_sphere->setMaterial(materials["basicShadowed"]);
		m_sphere->addGroupReference("shadowing");
		m_sphere->addGroupReference("shadowed");
	}

	virtual bool doInitialize() override
	{
		addComponent(m_sphere);
		return true;
	}

	void setRadius(double radius)
	{
		m_sphere->setRadius(radius);
	}

	void setPose(const RigidTransform3d& pose)
	{
		m_sphere->setPose(pose);
	}

private:
	std::shared_ptr<SurgSim::Graphics::OsgSphereRepresentation> m_sphere;
};


}

// Create an array of spheres
void addSpheres(std::shared_ptr<SurgSim::Framework::Scene> scene)
{
	double radius = 0.05;
	Vector3d origin (-1.0, 0.0, -1.0);
	Vector3d spacing (0.5, 0.5, 0.5);
	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			auto sphere = std::make_shared<SimpleSphere>("Sphere_"+std::to_string(i*3+j));
			sphere->setRadius(radius);
			Vector3d position = origin + Vector3d(spacing.array() * Vector3d(i,1.0,j).array());
			sphere->setPose(makeRigidTransform(Quaterniond::Identity(), position));
			scene->addSceneElement(sphere);
		}
	}

}

void createScene(std::shared_ptr<SurgSim::Framework::Runtime> runtime)
{
	auto scene = runtime->getScene();
	auto box = std::make_shared<SimpleBox>("Plane");
	box->setSize(3,0.01,3);
	box->setPose(RigidTransform3d::Identity());
	scene->addSceneElement(box);

	box = std::make_shared<SimpleBox>("Box 1");
	box->setSize(1.0,2.0,3.0);
	box->setPose(RigidTransform3d::Identity());
	scene->addSceneElement(box);

	scene->addSceneElement(createView("View", 0, 0, 1023, 768));
}


int main(int argc, char* argv[])
{
	const SurgSim::Framework::ApplicationData data("config.txt");

	materials["basicLit"] = loadMaterial(data, "Shaders/basic_lit");
	materials["basicUnlit"] = loadMaterial(data, "Shaders/basic_unlit");
	materials["basicShadowed"] = loadMaterial(data, "Shaders/shadowmap_vertexcolor");
	materials["depthMap"] = loadMaterial(data, "Shaders/depth_map");
	materials["shadowMap"] = loadMaterial(data, "Shaders/shadow_map");
	materials["default"] = materials["basic_lit"];

	auto runtime(std::make_shared<SurgSim::Framework::Runtime>());
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.5, 0.5, 0.5)));

	runtime->addManager(graphicsManager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	createScene(runtime);
	runtime->execute();

	return 0;
}