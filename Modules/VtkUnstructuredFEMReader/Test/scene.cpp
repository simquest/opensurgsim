#include <memory>
#include <boost/thread.hpp>

#include "../loadVtkUnstructuredData.h"

#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

using namespace SurgSim;


std::shared_ptr<Framework::SceneElement> createPlane()
{
	auto element = std::make_shared<Framework::BasicSceneElement>("Plane");

	auto physics = std::make_shared<Physics::FixedRepresentation>("Physics");
	physics->setShape(std::make_shared<Math::PlaneShape>());
	element->addComponent(physics);

	auto graphics = std::make_shared<Graphics::OsgPlaneRepresentation>("Graphics");
	element->addComponent(graphics);

	auto collision = std::make_shared<Physics::RigidCollisionRepresentation>("Collision");
	physics->setCollisionRepresentation(collision);
	element->addComponent(collision);

	return element;
}

std::shared_ptr<Framework::SceneElement> createFem(const std::string& filename)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("FEM");

	auto fem = std::make_shared<Fem3DTetrahedron>();
	loadVtkUnstructuredData(filename, fem);

	auto physics = std::make_shared<Physics::Fem3DRepresentation>("Physics");
	physics->setFem3D(fem);
	physics->setIntegrationScheme(Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER);
	element->addComponent(physics);

	//Copy the triangles from the fem into a surface mesh
	auto surfaceMesh = std::make_shared<Math::MeshShape>(*fem);

	auto collision = std::make_shared<Physics::DeformableCollisionRepresentation>("Collision");
	collision->setShape(surfaceMesh);
	physics->setCollisionRepresentation(collision);
	element->addComponent(collision);

	auto graphics = std::make_shared<Graphics::OsgMeshRepresentation>("Wire Frame");
	graphics->setShape(surfaceMesh);
	graphics->setDrawAsWireFrame(true);
	element->addComponent(graphics);

	auto graphicsUpdater = std::make_shared<Blocks::TransferPhysicsToGraphicsMeshBehavior>("Wire Frame Updater");
	graphicsUpdater->setSource(physics);
	graphicsUpdater->setTarget(graphics);
	element->addComponent(graphicsUpdater);

	return element;
}


int main(int argc, char* argv[])
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto scene = runtime->getScene();

	auto behaviorManager = std::make_shared<Framework::BehaviorManager>();
	auto graphicsManager = std::make_shared<Graphics::OsgManager>();
	auto physicsManager = std::make_shared<Physics::PhysicsManager>();
	runtime->addManager(behaviorManager);
	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);

	auto plane = createPlane();
	plane->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Math::Vector3d(0.0, 0.0, 0.0)));
	scene->addSceneElement(plane);

	auto fem = createFem("TestMesh.vtu");
	fem->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Math::Vector3d(0.0, 1.0, 0.0)));
	scene->addSceneElement(fem);

	auto view = std::make_shared<SurgSim::Graphics::OsgViewElement>("view");
	view->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Math::Vector3d(0.0, 0.5, 5.0)));
	scene->addSceneElement(view);

	runtime->execute();

	return 0;
}
