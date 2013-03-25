#include "SceneImplementation.h"

#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/Osg/ActorImplementation.h>
#include <SurgSim/Graphics/Osg/ViewImplementation.h>

using SurgSim::Graphics::Osg::ActorImplementation;
using SurgSim::Graphics::Osg::SceneImplementation;
using SurgSim::Graphics::Osg::ViewImplementation;

SceneImplementation::SceneImplementation() : SurgSim::Graphics::SceneImplementation(), m_root(new osg::Group())
{
	m_root->setName("Root");
}

SceneImplementation::~SceneImplementation()
{
}

bool SceneImplementation::doAddActor(std::shared_ptr<SurgSim::Graphics::ActorImplementation> actor)
{
	bool result = false;
	std::shared_ptr<ActorImplementation> osgActor = std::dynamic_pointer_cast<ActorImplementation>(actor);
	if (osgActor != nullptr)
	{
		result = m_root->addChild(osgActor->getNode());
	}
	return result;
}
bool SceneImplementation::doRemoveActor(std::shared_ptr<SurgSim::Graphics::ActorImplementation> actor)
{
	bool result = false;
	std::shared_ptr<ActorImplementation> osgActor = std::dynamic_pointer_cast<ActorImplementation>(actor);
	if (osgActor != nullptr)
	{
		result = m_root->removeChild(osgActor->getNode());
	}
	return result;
}

void SceneImplementation::doUpdate(double dt)
{
	for (auto it = m_views.begin(); it != m_views.end(); ++it)
	{
		(*it)->update(dt);
	}
}