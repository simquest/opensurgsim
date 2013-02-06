#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/SceneElement.h>

namespace SurgSim
{
namespace Framework
{

bool SurgSim::Framework::Scene::addSceneElement(std::shared_ptr<SceneElement> element)
{
	bool result = false;
	std::string name = element->getName();
	if (m_elements.find(name) == m_elements.end())
	{
		m_elements[name] = element;
		std::shared_ptr<Runtime> runtime = m_runtime.lock();
		if (runtime != nullptr)
		{
			std::shared_ptr<Runtime> runtime(m_runtime);
			runtime->addSceneElement(element);
		}
		result = true;
	}
	return result;
}

std::shared_ptr<SceneElement> Scene::getSceneElement(const std::string& name) const
{
	auto found = m_elements.find(name);
	std::shared_ptr<SceneElement> result;
	if (found != m_elements.end())
	{
		result = found->second;
	}
	return result;
}

void Scene::setRuntime(std::shared_ptr<Runtime> runtime)
{
	m_runtime = runtime;
}

const std::map<std::string,std::shared_ptr<SceneElement>>& Scene::getSceneElements() const
{
	return m_elements;
}

}; // namespace Framework
}; // namespace SurgSim

