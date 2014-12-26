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

#ifndef SURGSIM_PHYSICS_FEM1DREPRESENTATION_H
#define SURGSIM_PHYSICS_FEM1DREPRESENTATION_H

#include <memory>
#include <string>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Physics/FemRepresentation.h"

namespace SurgSim
{

namespace Physics
{
SURGSIM_STATIC_REGISTRATION(Fem1DRepresentation);

class FemPlyReaderDelegate;

/// Finite Element Model 1D is a fem built with 1D FemElement
///
/// The structure of a 1D model derives into a matrix structure of tri-diagonal block.
class Fem1DRepresentation : public FemRepresentation
{
public:
	/// Constructor
	/// \param name The name of the Fem1DRepresentation
	explicit Fem1DRepresentation(const std::string& name);

	/// Destructor
	virtual ~Fem1DRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem1DRepresentation);

	void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const SurgSim::Math::Vector& generalizedForce,
		const SurgSim::Math::Matrix& K = SurgSim::Math::Matrix(),
		const SurgSim::Math::Matrix& D = SurgSim::Math::Matrix()) override;

	/// Query the representation type
	/// \return the RepresentationType for this representation
	RepresentationType getType() const override;

protected:
	bool doWakeUp() override;

	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
			const SurgSim::Math::RigidTransform3d& transform) override;

private:
	std::shared_ptr<FemPlyReaderDelegate> getDelegate() override;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM1DREPRESENTATION_H
