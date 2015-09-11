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
#include "SurgSim/Blocks/DriveElementFromInputBehavior.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Devices/Devices.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Blocks/GraphicsUtilities.h"
#include "SurgSim/Blocks/ShadowMapping.h"

#include <boost/program_options.hpp>

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

std::shared_ptr<SurgSim::Graphics::ViewElement> createMonoView(
	const std::string& name,
	int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	std::array<int, 2> position = {x, y};
	std::array<int, 2> dimensions = {width, height};
	viewElement->getView()->setPosition(position);
	viewElement->getView()->setDimensions(dimensions);

	double aspectRatio = static_cast<double>(width) / static_cast<double>(height);
	viewElement->getCamera()->setPerspectiveProjection(60.0, aspectRatio, 0.01, 10.0);
	std::dynamic_pointer_cast<SurgSim::Graphics::OsgCamera>(viewElement->getCamera())->setMainCamera(true);
	/// It's an OsgViewElement, we have an OsgView, turn on mapping of uniform and attribute values
	std::dynamic_pointer_cast<SurgSim::Graphics::OsgView>(viewElement->getView())->setOsgMapsUniforms(true);

	// Move the camera from left to right over along the scene
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(
								Vector3d(-4.0, 1.0, -3.0),
								Vector3d(0.0, 0.0, 0.0),
								Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(
							  Vector3d(4.0, 1.0, -3.0),
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

	// result->addComponent(interpolator);

	return result;
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
		m_box->addGroupReference(SurgSim::Blocks::GROUP_SHADOW_CASTER);

		// Assign this to the pass for shadowed objects
		m_box->addGroupReference(SurgSim::Blocks::GROUP_SHADOW_RECEIVER);
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
		m_sphere->addGroupReference(SurgSim::Blocks::GROUP_SHADOW_CASTER);
		m_sphere->addGroupReference(SurgSim::Blocks::GROUP_SHADOW_RECEIVER);
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

void createScene(std::shared_ptr<SurgSim::Framework::Runtime> runtime, bool useStereo = false)
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
	sphere->setMaterial(materials["basicShadowed"]);

	scene->addSceneElement(sphere);
	if (useStereo == true)
	{
		// This will most like have to be edited to conform with the screen configuration
		// TargetScreen, or Dimensions and Location are candidates for being incorrect depending
		// on the system
		runtime->addSceneElements("StereoView.yaml");
	}
	else
	{
		scene->addSceneElement(createMonoView("View", 40, 40, 1024, 768));
	}

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Materials");
	element->addComponent(materials["shiny"]);
	element->addComponent(materials["texturedShadowed"]);
	element->addComponent(materials["basicShadowed"]);
	scene->addSceneElement(element);

	scene->addSceneElement(createLight());

	std::array<double, 6> lightProjection = { -2.6, 2.6, -2.6, 2.6, 4, 10};
	auto elements = SurgSim::Blocks::createShadowMapping(
						scene->getComponent("View", "Camera"),
						scene->getComponent("Light", "Light"),
						4096,
						1024,
						lightProjection,
						true,
						8.0,
						false);
	scene->addSceneElements(elements);
}


int main(int argc, char* argv[])
{
	using boost::program_options::value;

	bool useStereo = false;

	boost::program_options::options_description visible("Allowed options");
	visible.add_options()("help", "produce help message")
	("useStereo", value<bool>(&useStereo)->default_value(false), "Show scene via stereo");

	boost::program_options::variables_map variables;
	boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(visible).run(),
								  variables);
	boost::program_options::notify(variables);

	if (variables.count("help"))
	{
		std::cout << visible << "\n";
		return 1;
	}

	auto runtime(std::make_shared<SurgSim::Framework::Runtime>("config.txt"));
	auto data = runtime->getApplicationData();

	materials["basicLit"] = createMaterialWithShaders(*data, "Shaders/basic_lit");
	materials["basicUnlit"] = createMaterialWithShaders(*data, "Shaders/basic_unlit");
	materials["basicShadowed"] = createMaterialWithShaders(*data, "Shaders/s_mapping");
	materials["texturedUnlit"] = createMaterialWithShaders(*data, "Shaders/unlit_texture");
	materials["texturedShadowed"] = createMaterialWithShaders(*data, "Shaders/ds_mapping_material");
	materials["shiny"] = createMaterialWithShaders(*data, "Shaders/material");
	materials["default"] = materials["basic_lit"];

	auto graphics = std::make_shared<SurgSim::Graphics::OsgManager>();
	graphics->setRate(60);
	graphics->setMultiThreading(true);

	auto input = std::make_shared<SurgSim::Input::InputManager>();

	if (useStereo)
	{
		// Only interested in the oculus
		auto device = SurgSim::Devices::createDevice("SurgSim::Device::OculusDevice", "Oculus");

		if (device != nullptr)
		{
			input->addDevice(device);
		}
		else
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
					<< "Could not initialize Oculus Device, falling back to mono display.";
			useStereo = false;
		}
	}

	auto behaviors = std::make_shared<SurgSim::Framework::BehaviorManager>();

	runtime->addManager(graphics);
	runtime->addManager(behaviors);
	runtime->addManager(input);

	configureShinyMaterial();
	configureTexturedMaterial(runtime->getApplicationData()->findFile("Textures/CheckerBoard.png"));

	createScene(runtime, useStereo);

	runtime->execute();

	return 0;
}
