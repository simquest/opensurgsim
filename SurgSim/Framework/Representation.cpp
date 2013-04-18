#include "Representation.h"

SurgSim::Framework::Representation::Representation(const std::string& m_name) : Component(m_name)
{

}

SurgSim::Framework::Representation::~Representation()
{

}

bool SurgSim::Framework::Representation::doInitialize()
{
	return true;
}

bool SurgSim::Framework::Representation::doWakeUp()
{
	return true;
}


