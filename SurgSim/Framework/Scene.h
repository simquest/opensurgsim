#ifndef SURGSIM_SCENE_H
#define SURGSIM_SCENE_H

#include <memory>
#include <string>

#include "BasicThread.h"
#include "SceneElement.h"

namespace SurgSim
{
namespace Framework
{

class Runtime;

/// Scene. Basic Container for SceneElements 
class Scene
{
public:
	Scene() {};
	~Scene() {};

	/// Adds a scene element to 'element'.
	/// \param	name   	The name of the element.
	/// \param	element	The element.
	/// \return	true if it succeeds, false if it fails.
	bool addSceneElement(std::shared_ptr<SceneElement> element);

	/// Gets scene element with a given name
	/// \param	name	The name.
	/// \return	The scene element or nullptr if the element cannot be found.
	std::shared_ptr<SceneElement> getSceneElement(const std::string& name) const;

	/// Gets all the scene elements in the scene.
	/// \return	The scene elements.
	const std::map<std::string,std::shared_ptr<SceneElement>>& getSceneElements() const;

	/// Sets the runtime.
	/// \param	runtime	The runtime for this scene.
	void setRuntime(std::shared_ptr<Runtime> runtime);

private:
	std::weak_ptr<Runtime> m_runtime;

	std::map<std::string,std::shared_ptr<SceneElement>> m_elements;
};

}
}
#endif
