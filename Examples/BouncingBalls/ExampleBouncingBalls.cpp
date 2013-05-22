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

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/Actors/RigidActor.h>
#include <SurgSim/Physics/Actors/RigidActorParameters.h>
#include <SurgSim/Physics/Actors/SphereShape.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

#include <Examples/BouncingBalls/ConcreteSceneElement.h>

using SurgSim::Framework::SceneElement;
using SurgSim::Physics::Actor;
using SurgSim::Physics::RigidActor;
using SurgSim::Physics::SphereShape;
using SurgSim::Physics::RigidActorParameters;
using SurgSim::Physics::PhysicsManager;

///\file Example of how to put together a very simple demo of  balls colliding with each other
///		 dcd is used in a very simple manner to detect the collisions between the spheres


/// Simple behavior to show that the spheres are moving while we don't have graphics
class PrintoutBehavior : public SurgSim::Framework::Behavior
{
public:
	PrintoutBehavior(std::shared_ptr<RigidActor> actor) : m_actor(actor), Behavior("PrintoutBehavior") {}
	~PrintoutBehavior() {}

protected:
	virtual bool doInitialize() { return true;}
	virtual bool doWakeUp() {return true;}
	virtual void update(double dt) 
	{
		std::shared_ptr<SurgSim::Framework::Logger> logger = getRuntime()->getLogger("printout");
		SURGSIM_LOG_DEBUG(logger) << m_actor->getName() << ": " << m_actor->getPose().translation();
	}

private:
	std::shared_ptr<RigidActor> m_actor;
	
};

std::shared_ptr<SceneElement> createSphere(const std::string& name, const SurgSim::Math::Vector3d& position)
{
	std::shared_ptr<RigidActor> actor = std::make_shared<RigidActor>(name);

	RigidActorParameters params;
	params.setDensity(700.0); // Wood

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.01); // 1cm Sphere
	params.setShapeUsedForMassInertia(shape);

	actor->setInitialParameters(params);
	actor->setInitialPose(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond(), position));

	std::shared_ptr<SceneElement> sphereElement = std::make_shared<ConcreteSceneElement>(name);
	sphereElement->addComponent(actor);
	sphereElement->addComponent(std::make_shared<PrintoutBehavior>(actor));
	return sphereElement;
}


int main(int argc, char* argv[])
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());

	// std::shared_ptr<GraphicsManager> renderManager(new GraphicsManager());
	std::shared_ptr<PhysicsManager> physicsManager(new PhysicsManager());
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager(new SurgSim::Framework::BehaviorManager());

	// runtime->addManager(renderManager);
	runtime->addManager(physicsManager);
	runtime->addManager(behaviorManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());

	scene->addSceneElement(createSphere("sphere1", Vector3d(0.0,0.0,0.0)));

	runtime->setScene(scene);

	runtime->execute();

	return 0;
}
