#include "Scene.h"

#include "Actor.h"
#include "SceneImplementation.h"

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::Scene;
using SurgSim::Graphics::SceneImplementation;

Scene::Scene(std::shared_ptr<SceneImplementation> implementation) : m_implementation(implementation)
{
}

Scene::~Scene()
{
}

bool Scene::addActor(std::shared_ptr<Actor> actor)
{
	bool result = false;
	if (m_actors.find(actor->getName()) == m_actors.end())
	{
		m_actors[actor->getName()] = actor;
		return m_implementation->addActor(actor->getImplementation());
	}
	return result;
}
bool Scene::removeActor(const std::shared_ptr<Actor> actor)
{
	return removeActor(actor->getName());
}

bool Scene::removeActor(const std::string& name)
{
	bool result = false;
	auto it = m_actors.find(name);
	if (it != m_actors.end())
	{
		size_t count = m_actors.erase(name);
		result = (count == 1) && m_implementation->removeActor(it->second->getImplementation());
	}
	return result;
}

void Scene::doUpdate(double dt)
{
	for (auto it = m_actors.begin(); it != m_actors.end(); ++it)
	{
		it->second->update(dt);
	}
	m_implementation->update(dt);
}