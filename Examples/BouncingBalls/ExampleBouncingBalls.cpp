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
#include <boost/thread.hpp>

#include <SurgSim/Blocks/BasicSceneElement.h>
#include <SurgSim/Blocks/TransferPoseBehavior.h>
#include <SurgSim/Collision/RigidCollisionRepresentation.h>
#include <SurgSim/Collision/ShapeCollisionRepresentation.h>
#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgPlaneRepresentation.h>
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>
#include <SurgSim/Graphics/OsgUniform.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Math/DoubleSidedPlaneShape.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/SphereShape.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>

#include <Examples/BouncingBalls/AddRandomSphereBehavior.h>

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::AddRandomSphereBehavior;
using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgShader;
using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Graphics::OsgTexture2d;
using SurgSim::Graphics::OsgUniform;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector4f;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::PhysicsManager;

/// \file
///      Example of how to put together a very simple demo of balls colliding with each other.
///	 	 Discrete Collision Detection (dcd) is used to detect collisions between spheres.

/// Simple behavior to show that the spheres are moving while we don't have graphics.
/// \note A Behavior is a type of Component that causes changes or actions.
class PrintoutBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit PrintoutBehavior(std::shared_ptr<RigidRepresentation> representation) :
		Behavior("PrintoutBehavior"), m_representation(representation) {}
	~PrintoutBehavior() {}

	/// Perform per-period actions, i.e., what to do each "frame".
	/// \note Behavior::update() is called by ComponentManager::processBehaviors(), which is called by
	/// BehaviorManager::doUpdate() or Graphics::Manager::doUpdate() or PhysicsManager::doUpdate() depending on the
	/// output of Behavior::getTargetManagerType().  Those manager's \c doUpdate() functions are called by
	/// BasicThread() inside a \c while(running) loop.
	/// Managers (e.g., ComponentManager) are threads and run their own update loops.
	virtual void update(double dt)
	{
		// SURGSIM_LOG_DEBUG is a macro to ensure only messages of a certain threshold are output.
		SURGSIM_LOG_DEBUG(m_logger) << m_representation->getName() << ": " <<
								  m_representation->getPose().translation().transpose();
	}

protected:
	/// Allocate the internal structures.
	/// \return Success?
	/// \note Initialization is a two-step process.  First the ComponentManager calls initialize() on each Component,
	/// which calls doInitialize() to setup the internal structures.
	virtual bool doInitialize()
	{
		// If the named logger does not exist, the LoggerManager creates a new logger that uses the default output
		m_logger = Logger::getLogger("printout");
		// Set the threshold for display of messages.
		m_logger->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
		return true;
	}
	/// Setup foreign references. After this the Component is ready to update().
	/// \return Success?
	/// \note The second step of initialization. This function acquires any foreign references, all of which were
	/// allocated by the respective foreign objects' doInitialize().
	virtual bool doWakeUp()
	{
		return true;
	}

private:
	// A Representation is a type of Component that stores information.
	// A RigidRepresentation stores 6 degree-of-freedom (DOF) pose, force, torque, inertia, and compliance.
	std::shared_ptr<RigidRepresentation> m_representation;

	// A Logger can output to file or stream.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

/// Create a ViewElement to be added to the Scene.
/// \param name The name.
/// \param x The number of pixels from the left side of your display to place the window.
/// \param y The number of pixels from the top of your display to place the window.
/// \param width The width in pixels of the window.
/// \param height The height in pixels of the window.
/// \note The Scene is the container of all the SceneElements, e.g., a ball.  SceneElements are containers of
/// Components, e.g., RigidRepresentation and RigidCollisionRepresentation.
/// A ViewElement is a SceneElement that wraps a View.  A View is necessary to graphically visualize the Scene.
std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	// An OsgViewElement is an OSG implementation of a ViewElement.
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	return viewElement;
}

/// Creates a planar SceneElement with graphics, physics, and collision.
std::shared_ptr<SceneElement> createPlane(const SurgSim::Framework::ApplicationData& data, const std::string& name,
	const SurgSim::Math::RigidTransform3d& pose)
{
	// A FixedRepresentation has no motion or compliance. It does not change.
	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(name + " Physics");

	// A RigidTransform3d pose is the 6 degree-of-freedom (DOF) position and orientation.
	physicsRepresentation->setInitialPose(pose);

	// An OsgPlaneRepresentation is a Component containing the OSG-specific graphics information to display a plane.
	std::shared_ptr<OsgPlaneRepresentation> graphicsRepresentation =
		std::make_shared<OsgPlaneRepresentation>(name + " Graphics");
	// The initial graphical pose of the plane is set to match the initial physical pose.
	graphicsRepresentation->setInitialPose(pose);

	// A OsgMaterial is an OSG implementation of a Material.  Materials define visual appearance and contain Uniforms
	// and a Shader.  Uniforms represent values that are relatively constant, e.g., textures or position of a light.
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
	// An OsgShader is an OSG implementation of a Shader. Shaders are programs that determine how to render a geometry.
	std::shared_ptr<OsgShader> shader = std::make_shared<OsgShader>();

	// Create a Uniform for the RGBA color of the plane.
	std::shared_ptr<OsgUniform<Vector4f>> uniform = std::make_shared<OsgUniform<Vector4f>>("color");
	uniform->set(Vector4f(0.0f, 0.6f, 1.0f, 0.0f));
	material->addUniform(uniform);

	// This Shader sets the fragment's color to the value of the "color" uniform.
	shader->setFragmentShaderSource(
		"uniform vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_FragColor = color;\n"
		"}");
	material->setShader(shader);
	graphicsRepresentation->setMaterial(material);

	std::shared_ptr<DoubleSidedPlaneShape> planeShape = std::make_shared<DoubleSidedPlaneShape>();

	// Here the SceneElement for the plane is created, to which the various Components that collectively define the
	// plane are added.
	std::shared_ptr<SceneElement> planeElement = std::make_shared<BasicSceneElement>(name);
	planeElement->addComponent(physicsRepresentation);
	planeElement->addComponent(graphicsRepresentation);
	// The TransferPoseBehavior will copy the pose from the physics representation to the graphics representation.
	// The physics pose typically updates due to gravity and collisions, but this SceneElement will not move since it
	// uses a FixedRepresentation for physics.  Therefore, its physics pose will not change unless it is altered
	// outside of the physics/collision calculations, making this Component unnecessary for this example.  It is good
	// practice to ensure that the physics and graphics poses are synced anyway.
	planeElement->addComponent(std::make_shared<TransferPoseBehavior>("Physics to Graphics Pose",
							   physicsRepresentation, graphicsRepresentation));
	// ShapeCollisionRepresentation will use the provided Shape and physics to do collisions.  Collision detection
	// occurs in SurgSim::Physics::DcdCollision::doUpdate(), which uses the Shape.  Then the physics representations
	// (of the colliding pair) are used to generate constraints that the solver uses to calculate forces that will
	// un-collide the pair.  The entire process of collision detection, constraint generation, and solving is handled in
	// SurgSim::PhysicsManager::doUpdate().
	planeElement->addComponent(std::make_shared<SurgSim::Collision::ShapeCollisionRepresentation>
		("Plane Collision",planeShape, physicsRepresentation));

	// This Behavior will add balls to the Scene at random locations every few seconds.
	planeElement->addComponent(std::make_shared<AddRandomSphereBehavior>());

	return planeElement;
}


/// Creates a SceneElement of a rigid sphere with a graphic texture loaded from an image file.
/// \note This SceneElement does not have a collision Component.
/// \note Alternatively, a SurgSim::Blocks::SphereElement could be constructed and then its member variables altered.
std::shared_ptr<SceneElement> createEarth(const SurgSim::Framework::ApplicationData& data, const std::string& name,
	const SurgSim::Math::RigidTransform3d& pose)
{
	// A RigidRepresentation is for a non-deformable 6 degree-of-freedom (DOF) object with compliance and inertia.
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + " Physics");

	// A RigidRepresentationParameters defines physical parameters for a rigid body like mass/inertia/damping.
	RigidRepresentationParameters params;
	// Density determines inertia and compliance, which are used in the collision response.
	params.setDensity(5513.0); // Earth
	// Damping generates a force that opposes the velocity.
	params.setLinearDamping(0.1);

	// A SphereShape is a Shape for a sphere.  It has functions to find shape properties such as the mass center,
	// volume, and inertia. The constructor's argument is the radius in meters.
	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.5);
	params.setShapeUsedForMassInertia(shape);

	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<OsgSphereRepresentation> graphicsRepresentation =
		std::make_shared<OsgSphereRepresentation>(name + " Graphics");
	graphicsRepresentation->setRadius(shape->getRadius());
	graphicsRepresentation->setInitialPose(pose);

	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
	std::shared_ptr<OsgTexture2d> texture = std::make_shared<OsgTexture2d>();

	// findFile() will look in the folders specified by the ApplicationData.
	std::string image = data.findFile("Earth.png");
	SURGSIM_ASSERT(image != "") << "Could not find image file for sphere texture: " << image;
	SURGSIM_ASSERT(texture->loadImage(image)) << "Could not load image file for sphere texture: " << image;

	std::shared_ptr<OsgUniform<std::shared_ptr<OsgTexture2d>>> uniform =
		std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("diffuseMap");
	uniform->set(texture);
	material->addUniform(uniform);

	graphicsRepresentation->setMaterial(material);

	// Now create the SceneElement based on the physics and graphics.  Note there is no collision Component.
	std::shared_ptr<SceneElement> sphereElement = std::make_shared<BasicSceneElement>(name);
	sphereElement->addComponent(physicsRepresentation);
	sphereElement->addComponent(graphicsRepresentation);
	// By adding the PrintoutBehavior, the BehaviorManager will output this SceneElement's position each update.
	sphereElement->addComponent(std::make_shared<PrintoutBehavior>(physicsRepresentation));
	// Each time the BehaviorManager updates the Behaviors, transfer the pose from the physics Representation to the
	// graphics Representation.
	sphereElement->addComponent(std::make_shared<TransferPoseBehavior>("Physics to Graphics Pose",
		physicsRepresentation, graphicsRepresentation));
	return sphereElement;
}


int main(int argc, char* argv[])
{
	// The config file contains a list of folder locations which will be used to find resources, such as images, shader
	// code, physics descriptions, etc.
	const SurgSim::Framework::ApplicationData data("config.txt");

	// Here the various Managers are created.  Managers contain and update their type of Components. A Manager is a
	// sub-class of BasicThread, so each Manager runs in its own thread and can have its own target update rate.
	// The three types of Managers are:
	// a) Graphics::Manager to display the graphic scene from graphics representations (using graphics-shapes,
	// materials, shaders, etc.),
	// b) PhysicsManager to update the physics simulation based on physics representations (using physics-shapes,
	// fixed/rigid/FEM models, compliance, inertia, collision type, etc.), and
	// c) BehaviorManager for any actions not handled in the graphics or physics threads.
	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	// A Runtime is the top-level container for all of the Managers and a single Scene.
	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());

	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);

	// A Scene is a container for all of the SceneElements, which in turn contain their Components.
	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());

	scene->addSceneElement(createEarth(data, "earth1",
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,3.0,0.0))));

	scene->addSceneElement(createPlane(data, "plane1",
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0))));

	// Creates a ViewElement and adds it to the Scene.  A ViewElement is required for graphical display.
	scene->addSceneElement(createView("view1", 30, 30, 963, 707));

	// Place the camera.
	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	// Tell the Runtime which Scene to use. Since the Scene contains all of the Elements, a Runtime therefore has
	// access to all of the Components.
	runtime->setScene(scene);

	// Run the simulation, starting with initialize/startup of Managers and Components. For each Component of each
	// Element (Runtime::preprocessSceneElements) the Runtime tries to give access to the Component to each of the
	// Managers (Runtime::addComponents).  The Managers only accept the types of Components that they manage. Once the
	// Managers are running, the Runtime can be used to stop/step the Managers.
	// Runtime::execute() will block until one of the Managers quits.
	runtime->execute();

	return 0;
}
