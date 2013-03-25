#include "Scene.h"

#include "Actor.h"
#include "SceneImplementation.h"

using SurgSim::Physics::Actor;
using SurgSim::Physics::Scene;
using SurgSim::Physics::SceneImplementation;

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
	else
	{
		// TODO log this
// 		SURGSIM_LOG_WARNING(m_runtime.lock()->getLogger("runtime")) <<
// 			"Actor with name " << component->getName() <<
// 			" already exists in SceneImplementation " << getName() <<
// 			", did not add actor";
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

void Scene::doBeforeUpdate(double dt)
{
	for (auto it = m_actors.begin(); it != m_actors.end(); ++it)
	{
		it->second->beforeUpdate(dt);
	}
}
void Scene::doUpdate(double dt)
{
	m_implementation->update(dt);
}
void Scene::doAfterUpdate(double dt)
{
	for (auto it = m_actors.begin(); it != m_actors.end(); ++it)
	{
		it->second->afterUpdate(dt);
	}
}