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

#ifndef SURGSIM_PHYSICS_FEM2DREPRESENTATION_H
#define SURGSIM_PHYSICS_FEM2DREPRESENTATION_H

#include <memory>
#include <string>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2D.h"
#include "SurgSim/Physics/FemRepresentation.h"

namespace SurgSim
{
namespace DataStructures
{
struct IndexedLocalCoordinate;
struct Location;
}
namespace Framework
{
class Asset;
}
namespace Math
{
class OdeState;
}

namespace Physics
{
class Localization;

SURGSIM_STATIC_REGISTRATION(Fem2DRepresentation);

/// Finite Element Model 2D is a fem built with 2D FemElement
class Fem2DRepresentation : public FemRepresentation
{
public:
	/// Constructor
	/// \param name The name of the Fem2DRepresentation
	explicit Fem2DRepresentation(const std::string& name);

	/// Destructor
	virtual ~Fem2DRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem2DRepresentation);

	void loadFem(const std::string& fileName) override;

	/// Sets the fem mesh asset
	/// \param mesh The fem mesh to assign to this representation
	/// \exception SurgSim::Framework::AssertionFailure if mesh is nullptr or it's actual type is not Fem2D
	void setFem(std::shared_ptr<SurgSim::Framework::Asset> mesh);

	/// \return The fem mesh asset as a Fem2D
	std::shared_ptr<Fem2D> getFem() const;

	void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const SurgSim::Math::Vector& generalizedForce,
		const SurgSim::Math::Matrix& K = SurgSim::Math::Matrix(),
		const SurgSim::Math::Matrix& D = SurgSim::Math::Matrix()) override;

	std::shared_ptr<Localization> createLocalization(const SurgSim::DataStructures::Location& location) override;

protected:
	void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
			const SurgSim::Math::RigidTransform3d& transform) override;

	bool doInitialize() override;

private:
	/// Helper method: create a localization for a node-based IndexedLocalCoordinate
	/// \param location The IndexedLocalCoordinate pointing to the node index
	/// \return Localization of the node for this representation
	std::shared_ptr<Localization> createNodeLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location);

	/// Helper method: create a localization for an element-based IndexedLocalCoordinate (triangle)
	/// \param location The IndexedLocalCoordinate defining a point on the element mesh
	/// \return Localization of the point for this representation
	std::shared_ptr<Localization> createElementLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location);

	/// The Fem2DRepresentation's asset as a Fem2D
	std::shared_ptr<Fem2D> m_fem;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM2DREPRESENTATION_H
