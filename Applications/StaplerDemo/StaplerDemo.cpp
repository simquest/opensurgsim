// StaplerDemo.cpp : Defines the entry point for the console application.
//

#include <boost/thread.hpp>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/Manager.h>
#include <SurgSim/Graphics/Scene.h>
#include <SurgSim/Graphics/ViewComponent.h>
#include <SurgSim/Graphics/ViewElement.h>
#include <SurgSim/Graphics/Osg/SceneImplementation.h>
#include <SurgSim/Graphics/Osg/ViewImplementation.h>

using SurgSim::Framework::SceneElement;

int main(int argc, char* argv[])
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<SurgSim::Graphics::Manager> graphicsManager = std::make_shared<SurgSim::Graphics::Manager>(std::make_shared<SurgSim::Graphics::Scene>(std::make_shared<SurgSim::Graphics::Osg::SceneImplementation>()));
	
	runtime->addWorkerThread(graphicsManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene = std::make_shared<SurgSim::Framework::Scene>();
	scene->addSceneElement(std::make_shared<SurgSim::Graphics::ViewElement>("Default View", std::make_shared<SurgSim::Graphics::ViewComponent>("View", std::make_shared<SurgSim::Graphics::Osg::ViewImplementation>())));

	runtime->setScene(scene);

	runtime->execute();
	
	return 0;
}
