#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Framework
{

bool SceneElement::addComponent(std::shared_ptr<Component> component)
{
	bool result= false;
	if (m_components.find(component->getName()) == m_components.end())
	{
		m_components[component->getName()] = component;
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_runtime.lock()->getLogger("runtime")) <<
				"Component with name " << component->getName() <<
				" already exists on SceneElement " << getName() <<
				", did not add component";
	}
	return result;
}

bool SceneElement::removeComponent(std::shared_ptr<Component> component)
{
	return removeComponent(component->getName());
}

bool SceneElement::removeComponent(std::string name)
{
	bool result = false;
	if (m_components.find(name) != m_components.end())
	{
		size_t count = m_components.erase(name);
		result = (count == 1);
	}
	return result;
}

std::shared_ptr<Component> SceneElement::getComponent(std::string name) const
{
	std::shared_ptr<Component> result;

	auto findResult = m_components.find(name);
	if (findResult != m_components.end())
	{
		result = findResult->second;
	}
	return result;
}

bool SceneElement::initialize()
{
	bool result = true;
	for (auto it = m_components.begin(); it != m_components.end(); ++it)
	{
		bool componentInit = it->second->initialize();
		result = result && componentInit;
	}
	result = doInitialize() && result;
	return result;
}

bool SceneElement::wakeUp()
{
	bool result = true;
	for (auto it = m_components.begin(); it != m_components.end(); ++it)
	{
		bool componentInit = it->second->wakeUp();
		result = result && componentInit;
	}
	result = doWakeUp() && result;
	return result;
}

std::vector<std::shared_ptr<Component>> SceneElement::getComponents() const
{
	std::vector<std::shared_ptr<Component>> result(m_components.size());
	auto componentIt = m_components.begin();
	for (int i=0; componentIt != m_components.end(); ++componentIt, ++i)
	{
		result[i] = componentIt->second;
	}
	return result;
}

void SceneElement::setRuntime(std::shared_ptr<Runtime> runtime)
{
	m_runtime = runtime;
}

}; // namespace Framework
}; // namespace SurgSim




