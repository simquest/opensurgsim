#ifndef SURGSIM_FRAMEWORK_BEHAVIORMANAGER_H
#define SURGSIM_FRAMEWORK_BEHAVIORMANAGER_H

#include <memory>
#include <vector>

#include "BasicThread.h"

namespace SurgSim 
{
namespace Framework
{

class Behavior;

class BehaviorManager : public BasicThread
{
public:
	BehaviorManager();
	~BehaviorManager();
	virtual bool addComponent(std::shared_ptr<Component> component);
	virtual bool removeComponent(std::shared_ptr<Component> component);

private:
	//! \return false when the thread is done
	virtual bool doUpdate(double dt);

	virtual bool doInitialize() {return true;};
	virtual bool doStartUp() {return true;};

	std::vector<std::shared_ptr<Behavior>> m_behaviors;
};


}; // namespace Framework
}; // namespace SurgSim

#endif