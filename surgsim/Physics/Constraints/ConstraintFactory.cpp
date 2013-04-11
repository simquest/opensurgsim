#include "ConstraintFactory.h"

#include <SurgSim/Physics/Localizations/FixedLocalization.h>
#include <SurgSim/Physics/Localizations/Fem3dLocalization.h>
#include <SurgSim/Physics/Localizations/RigidLocalization.h>

#include <SurgSim/Physics/Constraints/Constraint.h>
#include <SurgSim/Physics/Constraints/Fem3dPointConstraintImplementation.h>
#include <SurgSim/Physics/Constraints/RigidPointConstraintImplementation.h>
#include <SurgSim/Physics/Constraints/FixedFrictionlessContactConstraintImplementation.h>
#include <SurgSim/Physics/Constraints/Fem3dFrictionlessContactConstraintImplementation.h>
#include <SurgSim/Physics/Constraints/RigidFrictionlessContactConstraintImplementation.h>

using SurgSim::Math::Vector4d;
using SurgSim::Physics::Constraint;
using SurgSim::Physics::ConstraintFactory;
using SurgSim::Physics::FixedLocalization;
using SurgSim::Physics::Fem3dLocalization;
using SurgSim::Physics::RigidLocalization;


ConstraintFactory::ConstraintFactory()
{
}

ConstraintFactory::~ConstraintFactory()
{
}

std::shared_ptr<Constraint> ConstraintFactory::makeFixedRigidFrictionlessContactConstraint(
    std::shared_ptr<FixedLocalization> fixedLocation, std::shared_ptr<RigidLocalization> rigidLocation,
    const Vector4d& normalPlane)
{
	std::shared_ptr<Constraint> constraint = m_constraintManager.get();
	constraint->reset();

	std::shared_ptr<FixedFrictionlessContactConstraintImplementation> fixedSide = m_fixedFrictionlessContactConstraintManager.get();
	fixedSide->reset();
	fixedSide->setLocation(fixedLocation);

	constraint->setSide(0, fixedSide);

	std::shared_ptr<RigidFrictionlessContactConstraintImplementation> rigidSide = m_rigidFrictionlessContactConstraintManager.get();
	rigidSide->reset();
	rigidSide->setLocation(rigidLocation);

	return constraint;
}

std::shared_ptr<Constraint> ConstraintFactory::makeFem3dRigidPointConstraint(
    std::shared_ptr<Fem3dLocalization> femLocation,
    std::shared_ptr<RigidLocalization> rigidLocation)
{
	std::shared_ptr<Constraint> constraint = m_constraintManager.get();
	constraint->reset();

	std::shared_ptr<Fem3dPointConstraintImplementation> femSide = m_fem3dPointConstraintManager.get();
	femSide->reset();
	femSide->setLocation(femLocation);

	constraint->setSide(0, fixedSide);

	std::shared_ptr<RigidPointConstraintImplementation> rigidSide = m_rigidPointConstraintManager.get();
	rigidSide->reset();
	rigidSide->setLocation(rigidLocation);

	return constraint;
}

std::shared_ptr<Constraint> ConstraintFactory::makeFem3dRigidFrictionlessContactConstraint(
    std::shared_ptr<Fem3dLocalization> femLocation,
    std::shared_ptr<RigidLocalization> rigidLocation,
    const Vector4d& normalPlane)
{
	std::shared_ptr<Constraint> constraint = m_constraintManager.get();
	constraint->reset();

	std::shared_ptr<Fem3dFrictionlessContactConstraintImplementation> femSide = m_fem3dFrictionlessContactConstraintManager.get();
	femSide->reset();
	femSide->setLocation(femLocation);

	constraint->setSide(0, fixedSide);

	std::shared_ptr<RigidFrictionlessContactConstraintImplementation> rigidSide = m_rigidFrictionlessContactConstraintManager.get();
	rigidSide->reset();
	rigidSide->setLocation(rigidLocation);

	return constraint;
}