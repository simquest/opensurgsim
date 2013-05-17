#ifndef SURGSIM_PHYSICS_PHYSICSMANAGER_H
#define SURGSIM_PHYSICS_PHYSICSMANAGER_H

#include <memory>
#include <vector>

#include <SurgSim/Framework/ComponentManager.h>


namespace SurgSim 
{
namespace Framework
{
	class Logger;
}
namespace Physics
{

class RigidActorBase;
class FreeMotionStep;

class PhysicsManager : public SurgSim::Framework::ComponentManager
{
public:
	PhysicsManager();
	~PhysicsManager();

	bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);
	bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);

protected:

	virtual bool doInitialize();
	virtual bool doUpdate(double dt);

private:

	std::shared_ptr< std::vector<std::shared_ptr<RigidActorBase>> > m_rigidActors;
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	std::unique_ptr<FreeMotionStep> m_freeMotionStep;

};


}; // namespace Physics
}; // namespace SurgSim

#endif