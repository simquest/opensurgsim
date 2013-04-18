#ifndef SURGSIM_REPRESENTATION_H
#define SURGSIM_REPRESENTATION_H

#include <memory>

#include "Component.h"

namespace SurgSim
{
namespace Framework
{

class SceneElement;

class Representation : public Component
{
public:
	Representation(const std::string& m_name);;
	virtual ~Representation();
private:
	virtual bool doInitialize();
	virtual bool doWakeUp();
};

}
}

#endif

