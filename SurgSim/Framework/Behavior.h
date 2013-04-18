#ifndef SURGSIM_FRAMEWORK_BEHAVIOR_H
#define SURGSIM_FRAMEWORK_BEHAVIOR_H

#include "Component.h"

namespace SurgSim
{
namespace Framework
{

class Behavior: public Component
{
public:
	Behavior(std::string name) : Component(name) {};
	virtual ~Behavior() {};

	virtual void update(double dt) = 0;
};

}; //namespace Framework
}; //namespace SurgSim

#endif
