#ifndef SURGSIM_COMPONENT_H
#define SURGSIM_COMPONENT_H

#include <string>

#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Framework
{

// Forward References
class SceneElement;

/// Component is the main interface class to pass information to the system managers each will decide
/// whether to handle a component of a given type or not. Components will get initialized by having 
/// doInit(), and doWakeUp() called in succession, all components together will have doInit() called before
/// any component will recieve doWakeUp()
class Component
{
public:
	Component(const std::string& name) : m_name(name), m_didInit(false), m_didWakeUp(false) {};
	virtual ~Component(void) {};

	/// Gets the name.
	/// \return	The name.
	std::string getName()
	{
		return m_name;
	};

	bool doInit() 
	{
		SURGSIM_ASSERT(! m_didInit) << "Double initialisation called on component " << getName();
		m_didInit = true;
		return init();
	};
	bool doWakeUp() 
	{
		SURGSIM_ASSERT(! m_didWakeUp) << "Double wakeup called on component " << getName();
		m_didWakeUp = true;
		return wakeUp();
	};

private:
	std::string m_name;

	virtual bool init() = 0;
	virtual bool wakeUp() = 0;

	bool m_didInit;
	bool m_didWakeUp;
};

}
}
#endif
