#ifndef SURGSIM_SCENEELEMENT_H
#define SURGSIM_SCENEELEMENT_H

#include <string>
#include <memory>
#include <algorithm>
#include <unordered_map>

namespace SurgSim
{
namespace Framework
{

class Component;
class Runtime;

/// SceneElement is the basic part of a scene, it is a container of components. SceneElements 
/// doInit() will be called on all SceneElements before they are integrated into the system, 
/// doWakeup() will be called after all doInit() have been completed and all the static information
/// has been initialised
class SceneElement
{

public:
	SceneElement(std::string name) : m_name(name) {};
	virtual ~SceneElement() {};

	/// Adds a component
	/// \param	component	The component.
	/// \return	true if it succeeds, false if it fails.
	bool addComponent(std::shared_ptr<Component> component);

	/// Removes the give component.
	/// \param	component	The component.
	/// \return	true if it succeeds, false if it fails or the component cannot be found.
	bool removeComponent(std::shared_ptr<Component> component);

	/// Removes the component described by name.
	/// \param	name	The name.
	/// \return	true if it succeeds, false if it fails or the component cannot be found.
	bool removeComponent(std::string name);

	/// Gets the component identified by name.
	/// \param	name	The name.
	/// \return	The component or nullptr if the component cannot be found.
	std::shared_ptr<Component> getComponent(std::string name) const;

	/// Gets all the components of this SceneElement.
	/// \return	The components.
	std::vector<std::shared_ptr<Component>> getComponents() const;

	/// Executes the initialize operation.
	/// \return	true if it succeeds, false if it fails.
	bool doInit();

	/// Executes the wake up operation.
	/// \return	true if it succeeds, false if it fails.
	bool doWakeUp();

	/// \return	The name.
	std::string getName() const 
	{
		return m_name;
	}

	/// Sets the Runtime.
	/// \param runtime Pointer to the runtime.
	void setRuntime(std::shared_ptr<Runtime> runtime);

private:

	std::string m_name;
	std::unordered_map<std::string, std::shared_ptr<Component>> m_components;
	std::weak_ptr<Runtime> m_runtime;
	virtual bool init() = 0;
	virtual bool wakeUp() = 0;

};

}
}
#endif