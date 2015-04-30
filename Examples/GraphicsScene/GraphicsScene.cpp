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

#include <osg/Matrix>
#include <osg/Camera>

#include "SurgSim/Blocks/PoseInterpolator.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Blocks/GraphicsUtilities.h"

using SurgSim::Framework::Logger;
using SurgSim::Graphics::OsgTextureUniform;
using SurgSim::Graphics::OsgTexture2d;
using SurgSim::Graphics::OsgUniform;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Matrix44f;

/// \file
/// This example creates a simple graphics scene and use the RenderPass object to show
/// a simple shadowing algorithm. For the algorithm see http://en.wikipedia.org/wiki/Shadow_mapping
/// There are two preprocessing passes that use specific program, plus a main pass that renders the
/// objects using a program that can process the output from the preprocessing steps.
/// Each program output becomes the input for the next shader, some parameters from other scene elements
/// is passed into the program as uniforms.
/// All of the information is kept up to date using a \sa TransferPropertiesBehavior
/// Both the light and the main camera are being moved through a \sa PoseInterpolator to demonstrate
/// dynamic changes and how to handle them in the rendering pipeline

namespace
{

std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>> materials;

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	std::array<int, 2> position = {x, y};
	std::array<int, 2> dimensions = {width, height};
	viewElement->getView()->setPosition(position);
	viewElement->getView()->setDimensions(dimensions);

	/// It's an OsgViewElement, we have an OsgView, turn on mapping of uniform and attribute values
	std::dynamic_pointer_cast<SurgSim::Graphics::OsgView>(viewElement->getView())->setOsgMapsUniforms(false);

	// Move the camera from left to right over along the scene
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("Interpolator_2");
	RigidTransform3d from = makeRigidTransform(
								Vector3d(-4.0, 2.0, -4.0),
								Vector3d(0.0, 0.0, 0.0),
								Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(
							  Vector3d(4.0, 2.0, -4.0),
							  Vector3d(0.0, 0.0, 0.0),
							  Vector3d(0.0, 1.0, 0.0));
	interpolator->setTarget(viewElement);
	interpolator->setStartingPose(from);
	interpolator->setDuration(10.0);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	viewElement->setPose(from);

	viewElement->enableManipulator(true);
	// viewElement->addComponent(interpolator);

	return viewElement;
}

std::shared_ptr<SurgSim::Graphics::OsgMaterial> createMaterialWithShaders(
	const SurgSim::Framework::ApplicationData& data,
	const std::string& name)
{

	auto program = SurgSim::Graphics::loadProgram(data, name);

	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material;
	if (program != nullptr)
	{
		material = std::make_shared<SurgSim::Graphics::OsgMaterial>(name);
		material->setProgram(program);
	}

	return material;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createLight()
{
	auto result = std::make_shared<SurgSim::Framework::BasicSceneElement>("Light");

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	result->addComponent(light);

	// Move the light from left to right over along the scene
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(Vector3d(5.0, 3.0, -5.0),
							Vector3d(0.0, 0.0, 0.0),
							Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(Vector3d(-5.0, 3.0, -5.0),
						  Vector3d(0.0, 0.0, 0.0),
						  Vector3d(0.0, 1.0, 0.0));
	interpolator->setTarget(result);
	interpolator->setStartingPose(from);
	interpolator->setDuration(10.0);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	result->setPose(from);

	result->addComponent(interpolator);

	return result;
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
	materials["depthMap"]->getProgram()->setGlobalScope(true);
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
	materials["shadowMap"]->getProgram()->setGlobalScope(true);
	pass->setMaterial(materials["shadowMap"]);
	return pass;
}

void configureShinyMaterial()
{
	// This will change the shared material
	auto material = materials["shiny"];

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", SurgSim::Math::Vector4f(0.8, 0.8, 0.1, 1.0));

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", SurgSim::Math::Vector4f(0.9, 0.9, 0.1, 1.0));

	material->addUniform("float", "shininess");
	material->setValue("shininess", 32.0f);
}

void configureTexturedMaterial(const std::string& filename)
{
	auto material = materials["texturedShadowed"];
	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", SurgSim::Math::Vector4f(1.0, 1.0, 1.0, 1.0));

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", SurgSim::Math::Vector4f(1.0, 1.0, 1.0, 1.0));

	material->addUniform("float", "shininess");
	material->setValue("shininess", 32.0f);

	auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
	texture->loadImage(filename);

	material->addUniform("sampler2D", "diffuseMap");
	material->setValue("diffuseMap", texture);
}

/// A simple box as a scenelement
class SimpleBox : public SurgSim::Framework::BasicSceneElement
{
public:
	explicit SimpleBox(const std::string& name) : BasicSceneElement(name)
	{
		m_box = std::make_shared<SurgSim::Graphics::OsgBoxRepresentation>(getName() + " Graphics");

		// The material that this object uses
		m_box->setMaterial(materials["basicShadowed"]);

		// Assign this to the pass for shadowing objects
		m_box->addGroupReference("shadowing");

		// Assign this to the pass for shadowed objects
		m_box->addGroupReference("shadowed");
	}

	bool doInitialize() override
	{
		addComponent(m_box);
		return true;
	}

	void setSize(double width, double height, double length)
	{
		m_box->setSizeXYZ(width, height, length);
	}

	void setMaterial(const std::shared_ptr<SurgSim::Graphics::Material> material)
	{
		m_box->setMaterial(material);
	}

private:
	std::shared_ptr<SurgSim::Graphics::OsgBoxRepresentation> m_box;
};


class SimpleSphere : public SurgSim::Framework::BasicSceneElement
{
public:
	explicit SimpleSphere(const std::string& name) : BasicSceneElement(name)
	{
		m_sphere = std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>(getName() + " Graphics");
		m_sphere->setMaterial(materials["basicShadowed"]);
		m_sphere->addGroupReference("shadowing");
		m_sphere->addGroupReference("shadowed");
	}

	bool doInitialize() override
	{
		addComponent(m_sphere);
		return true;
	}

	void setRadius(double radius)
	{
		m_sphere->setRadius(radius);
	}

	void setMaterial(const std::shared_ptr<SurgSim::Graphics::Material> material)
	{
		m_sphere->setMaterial(material);
	}

private:
	std::shared_ptr<SurgSim::Graphics::OsgSphereRepresentation> m_sphere;
};


}

// Create an array of spheres
void addSpheres(std::shared_ptr<SurgSim::Framework::Scene> scene)
{
	double radius = 0.05;
	Vector3d origin(-1.0, 0.0, -1.0);
	Vector3d spacing(0.5, 0.5, 0.5);
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			auto sphere = std::make_shared<SimpleSphere>("Sphere_" + std::to_string(static_cast<int64_t>(i * 3 + j)));
			sphere->setRadius(radius);
			Vector3d position = origin + Vector3d(spacing.array() * Vector3d(static_cast<double>(i),
												  1.0,
												  static_cast<double>(j)).array());
			sphere->setPose(makeRigidTransform(Quaterniond::Identity(), position));
			scene->addSceneElement(sphere);
		}
	}

}

std::shared_ptr<SurgSim::Graphics::ScreenSpaceQuadRepresentation> makeDebugQuad(
	const std::string& name,
	std::shared_ptr<SurgSim::Graphics::Texture> texture,
	double x, double y, double width, double height)
{
	auto result = std::make_shared<SurgSim::Graphics::OsgScreenSpaceQuadRepresentation>(name);
	result->setTexture(texture);
	result->setSize(width, height);
	result->setLocation(x, y);
	return result;
}


void createScene(std::shared_ptr<SurgSim::Framework::Runtime> runtime)
{
	auto scene = runtime->getScene();
	auto box = std::make_shared<SimpleBox>("Plane");
	box->setSize(3.0, 0.01, 3.0);
	box->setPose(RigidTransform3d::Identity());
	box->setMaterial(materials["texturedShadowed"]);
	scene->addSceneElement(box);

	box = std::make_shared<SimpleBox>("Box 1");
	box->setSize(0.25, 1.0, 0.25);
	box->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(1.0, 0.5, -1.0)));
	scene->addSceneElement(box);

	addSpheres(scene);

	auto sphere = std::make_shared<SimpleSphere>("Shiny Sphere");
	sphere->setRadius(0.25);
	sphere->setMaterial(materials["shiny"]);

	scene->addSceneElement(sphere);

	std::shared_ptr<SurgSim::Graphics::ViewElement> viewElement = createView("View", 40, 40, 1024, 768);
	scene->addSceneElement(viewElement);
	auto mainCamera = viewElement->getCamera();

	// This behavior is responsible to keep all values updated, in this example most targets
	// will be uniforms that are used in shaders
	auto copier =  std::make_shared<SurgSim::Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);
	viewElement->addComponent(copier);
	viewElement->addComponent(materials["shiny"]);
	viewElement->addComponent(materials["texturedShadowed"]);


	auto lightElement = createLight();
	scene->addSceneElement(lightElement);

	auto lightMapPass = createLightMapPass();
	scene->addSceneElement(lightMapPass);

	// connect the light pose and the light map camera pose, so when the light moves,
	// this camera will move as well
	copier->connect(lightElement->getPoseComponent(), "Pose", lightMapPass->getPoseComponent(), "Pose");

	auto shadowMapPass = createShadowMapPass();
	shadowMapPass->getCamera()->setOrthogonalProjection(-4, 4, -2, 2, 0, 4);

	// The following three uniforms in the shadowMapPass, carry the information from the
	// lightMapPass. They are used to project the incoming point into the space of the lightMap
	// The view matrix of the camera used to render the light map
	shadowMapPass->getMaterial()->addUniform("mat4", "lightViewMatrix");
	copier->connect(lightMapPass->getCamera(), "FloatViewMatrix",
					shadowMapPass->getMaterial(), "lightViewMatrix");

	// The projection matrix of the camera used to render the light map
	shadowMapPass->getMaterial()->addUniform("mat4", "lightProjectionMatrix");
	copier->connect(lightMapPass->getCamera(), "FloatProjectionMatrix",
					shadowMapPass->getMaterial(), "lightProjectionMatrix");

// The inverse view matrix of the camera used to render the light map
// HS-2016-jun-04 Leave commented for now, this exposes a bug in terms of timing with the
// copy behaviors, while the inverseViewMatrix uniform is now available from the camera, using this
// form causes rendering artifacts, possibly due to a mismatch between the viewMatrix and the inverse
// 	auto inverseViewMatrix = std::make_shared<OsgUniform<Matrix44f>>("inverseViewMatrix");
// 	shadowMapPass->getMaterial()->addUniform(inverseViewMatrix);
// 	copier->connect(shadowMapPass->getCamera(), "FloatInverseViewMatrix",
// 					shadowMapPass->getMaterial() , "inverseViewMatrix");

	// Get the result of the lightMapPass and pass it on to the shadowMapPass, because it is used
	// in a pass we ask the system to use a higher than normal texture unit (in this case 8) for
	// this texture, this prevents the texture from being overwritten by other textures
	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material;
	material = std::dynamic_pointer_cast<SurgSim::Graphics::OsgMaterial>(shadowMapPass->getMaterial());

	material->addUniform("sampler2D", "encodedLightDepthMap");
	material->setValue("encodedLightDepthMap", lightMapPass->getRenderTarget()->getColorTarget(0));
	material->getUniform("encodedLightDepthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));

	// Make the camera in the shadowMapPass follow the main camera that is being used to render the
	// whole scene
	copier->connect(viewElement->getPoseComponent(), "Pose", shadowMapPass->getPoseComponent(), "Pose");
	copier->connect(mainCamera, "ProjectionMatrix", shadowMapPass->getCamera() , "ProjectionMatrix");
	scene->addSceneElement(shadowMapPass);


	// Put the result of the last pass into the main camera to make it accessible
	material = std::make_shared<SurgSim::Graphics::OsgMaterial>("material");

	material->addUniform("sampler2D", "shadowMap");
	material->setValue("shadowMap", shadowMapPass->getRenderTarget()->getColorTarget(0));
	material->getUniform("shadowMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	mainCamera->setMaterial(material);
	viewElement->addComponent(material);

	auto debug = std::make_shared<SurgSim::Framework::BasicSceneElement>("debug");
	debug->addComponent(makeDebugQuad("light",
									  lightMapPass->getRenderTarget()->getColorTarget(0), 0, 0, 256, 256));
	debug->addComponent(makeDebugQuad("shadow",
									  shadowMapPass->getRenderTarget()->getColorTarget(0), 1024 - 256, 0, 256, 256));
	scene->addSceneElement(debug);
}

void createSceneWithUtilities(std::shared_ptr<SurgSim::Framework::Runtime> runtime)
{
	auto scene = runtime->getScene();
	auto box = std::make_shared<SimpleBox>("Plane");
	box->setSize(3.0, 0.01, 3.0);
	box->setPose(RigidTransform3d::Identity());
	box->setMaterial(materials["texturedShadowed"]);
	scene->addSceneElement(box);

	box = std::make_shared<SimpleBox>("Box 1");
	box->setSize(0.25, 1.0, 0.25);
	box->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(1.0, 0.5, -1.0)));
	scene->addSceneElement(box);

	addSpheres(scene);

	auto sphere = std::make_shared<SimpleSphere>("Shiny Sphere");
	sphere->setRadius(0.25);
	sphere->setMaterial(materials["shiny"]);

	scene->addSceneElement(sphere);

	std::shared_ptr<SurgSim::Graphics::ViewElement> viewElement = createView("View", 40, 40, 1024, 768);
	scene->addSceneElement(viewElement);
	auto mainCamera = viewElement->getCamera();

	viewElement->addComponent(materials["shiny"]);
	viewElement->addComponent(materials["texturedShadowed"]);

	auto lightElement = createLight();
	scene->addSceneElement(lightElement);


	SurgSim::Blocks::setupShadowMapping(materials, scene);
}


int main(int argc, char* argv[])
{
	auto runtime(std::make_shared<SurgSim::Framework::Runtime>("config.txt"));
	auto data = runtime->getApplicationData();

	materials["basicLit"] = createMaterialWithShaders(*data, "Shaders/basic_lit");
	materials["basicUnlit"] = createMaterialWithShaders(*data, "Shaders/basic_unlit");
	materials["basicShadowed"] = createMaterialWithShaders(*data, "Shaders/s_mapping");
	materials["texturedShadowed"] = createMaterialWithShaders(*data, "Shaders/ds_mapping_material");
	materials["shiny"] = createMaterialWithShaders(*data, "Shaders/material");
	materials["depthMap"] = createMaterialWithShaders(*data, "Shaders/depth_map");
	materials["shadowMap"] = createMaterialWithShaders(*data, "Shaders/shadow_map");
	materials["default"] = materials["basic_lit"];
	materials["horizontalBlur"] = createMaterialWithShaders(*data, "Shaders/horizontalBlurPass");
	materials["verticalBlur"] = createMaterialWithShaders(*data, "Shaders/verticalBlurPass");

	auto graphics = std::make_shared<SurgSim::Graphics::OsgManager>();
	graphics->setRate(60.0);
	runtime->addManager(graphics);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	configureShinyMaterial();
	configureTexturedMaterial(runtime->getApplicationData()->findFile("Textures/CheckerBoard.png"));

	createSceneWithUtilities(runtime);

	runtime->execute();

	return 0;
}
