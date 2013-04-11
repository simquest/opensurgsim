// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_PHYSICS_CONSTRAINT_FACTORY_H
#define SURGSIM_PHYSICS_CONSTRAINT_FACTORY_H

#include <SurgSim/Framework/ReuseManager.h>
#include <SurgSim/Math/Vector4d.h>

#include <memory>

namespace SurgSim
{

namespace Physics
{

class Constraint;
class FixedLocalization;
class FixedFrictionlessContactConstraintImplementation;
class Fem3dLocalization;
class Fem3dPointConstraintImplementation;
class Fem3dFrictionlessContactConstraintImplementation;
class RigidLocalization;
class RigidPointConstraintImplementation;
class RigidFrictionlessContactConstraintImplementation;

class ConstraintFactory
{
public:
	ConstraintFactory();
	virtual ~ConstraintFactory();

	std::shared_ptr<Constraint> makeFixedRigidFrictionlessContactConstraint(std::shared_ptr<FixedLocalization> fixedLocation,
		std::shared_ptr<RigidLocalization> rigidLocation, const SurgSim::Math::Vector4d& normalPlane);

	std::shared_ptr<Constraint> makeFem3dRigidPointConstraint(std::shared_ptr<Fem3dLocalization> femLocation,
		std::shared_ptr<RigidLocalization> rigidLocation);

	std::shared_ptr<Constraint> makeFem3dRigidFrictionlessContactConstraint(std::shared_ptr<Fem3dLocalization> femLocation,
		std::shared_ptr<RigidLocalization> rigidLocation, const SurgSim::Math::Vector4d& normalPlane);

private:
	SurgSim::Framework::ReuseManager<Constraint> m_constraintManager;

	SurgSim::Framework::ReuseManager<Fem3dPointConstraintImplementation> m_femPointConstraintManager;
	SurgSim::Framework::ReuseManager<RigidPointConstraintImplementation> m_rigidPointConstraintManager;

	SurgSim::Framework::ReuseManager<Fem3dFrictionlessContactConstraintImplementation> m_femFrictionlessContactConstraintManager;
	SurgSim::Framework::ReuseManager<FixedFrictionlessContactConstraintImplementation> m_fixedFrictionlessContactConstraintManager;
	SurgSim::Framework::ReuseManager<RigidFrictionlessContactConstraintImplementation> m_rigidFrictionlessContactConstraintManager;
	SurgSim::Framework::ReuseManager<ContactConstraintData> m_contactConstraintDataManager;	
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINT_FACTORY_H
