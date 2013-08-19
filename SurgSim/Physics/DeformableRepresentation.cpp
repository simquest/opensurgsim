#include <SurgSim/Physics/DeformableRepresentation.h>

namespace SurgSim
{

namespace Physics
{

DeformableRepresentation::DeformableRepresentation(const std::string& name) :
Representation(name)
{
	m_initialPose.setIdentity();
}

DeformableRepresentation::~DeformableRepresentation()
{
}

void DeformableRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
}

const SurgSim::Math::RigidTransform3d& DeformableRepresentation::getInitialPose() const 
{
	return m_initialPose;
}

void DeformableRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_ASSERT(true) << "setPose has been called on a physics DeformableRepresentation";
}

const SurgSim::Math::RigidTransform3d& DeformableRepresentation::getPose() const
{
	static const SurgSim::Math::RigidTransform3d staticLocalId = SurgSim::Math::RigidTransform3d::Identity();
	return staticLocalId;
}

void DeformableRepresentation::resetState()
{
	Representation::resetState();

	m_currentState  = m_initialState;
	m_previousState = m_initialState;
	m_finalState    = m_initialState;
}

void DeformableRepresentation::setInitialState(const DeformableRepresentationState& state)
{
	m_initialState  = state;
	m_currentState  = state;
	m_previousState = state;
	m_finalState    = state;
}

const DeformableRepresentationState& DeformableRepresentation::getInitialState() const
{
	return m_initialState;
}

const DeformableRepresentationState& DeformableRepresentation::getCurrentState() const
{
	return m_currentState;
}

const DeformableRepresentationState& DeformableRepresentation::getPreviousState() const
{
	return m_previousState;
}

const DeformableRepresentationState& DeformableRepresentation::getFinalState() const
{
	return m_finalState;
}

}; // namespace Physics

}; // namespace SurgSim
