
#include "BehaviorManager.h"

#include <memory>
#include <vector>

#include "Component.h"
#include "Behavior.h"


SurgSim::Framework::BehaviorManager::BehaviorManager() : BasicThread ("Behavior Manager")
{

}

SurgSim::Framework::BehaviorManager::~BehaviorManager()
{

}

bool SurgSim::Framework::BehaviorManager::addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = false;
	std::shared_ptr<Behavior> behavior = std::dynamic_pointer_cast<Behavior>(component);
	if (behavior != nullptr && find(m_behaviors.begin(), m_behaviors.end(),behavior) == m_behaviors.end())
	{
		m_behaviors.push_back(behavior);
		result = true;
	}
	return result;
}

bool SurgSim::Framework::BehaviorManager::removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = false;
	std::shared_ptr<Behavior> behavior = std::dynamic_pointer_cast<Behavior>(component);
	if (behavior != nullptr && m_behaviors.size() != 0)
	{
		auto found = std::find(m_behaviors.begin(), m_behaviors.end(), behavior);
		if (found != m_behaviors.end())
		{
			m_behaviors.erase(found);
			result = true;
		}
	}
	return result;
}

bool SurgSim::Framework::BehaviorManager::doUpdate(double dt)
{
	auto endIt = m_behaviors.end();
	auto it = m_behaviors.begin();
	for (;it != endIt; ++it)
	{
		(*it)->update(dt);
	}
	return true;
}