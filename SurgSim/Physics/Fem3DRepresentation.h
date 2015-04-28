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

#ifndef SURGSIM_PHYSICS_FEM3DREPRESENTATION_H
#define SURGSIM_PHYSICS_FEM3DREPRESENTATION_H

#include <memory>
#include <string>
#include <unordered_map>

#include "SurgSim/Framework/FrameworkConvert.h"
//#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Physics/FemRepresentation.h"

namespace SurgSim
{

namespace Math
{
class MeshShape;
}

namespace Physics
{
SURGSIM_STATIC_REGISTRATION(Fem3DRepresentation);

class FemPlyReaderDelegate;

/// Finite Element Model 3D is a fem built with 3D FemElement
class Fem3DRepresentation : public FemRepresentation
{
public:
	/// Constructor
	/// \param name The name of the Fem3DRepresentation
	explicit Fem3DRepresentation(const std::string& name);

	/// Destructor
	virtual ~Fem3DRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem3DRepresentation);

	void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
			const SurgSim::Math::Vector& generalizedForce,
			const SurgSim::Math::Matrix& K = SurgSim::Math::Matrix(),
			const SurgSim::Math::Matrix& D = SurgSim::Math::Matrix()) override;

	std::shared_ptr<Localization> createLocalization(const SurgSim::DataStructures::Location&) override;

protected:
	bool doWakeUp() override;

	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
			const SurgSim::Math::RigidTransform3d& transform) override;

private:
	std::shared_ptr<FemPlyReaderDelegate> getDelegate() override;

	/// Produces a mapping from the provided mesh's triangle ids to this object's fem element ids. The mesh's vertices
	/// must be identical to this object's fem element nodes.
	/// \param mesh The mesh used to produce the mapping.
	/// \return A map from the mesh's triangle ids to this object's fem elements.
	std::unordered_map<size_t, size_t> createTriangleIdToElementIdMap(
			std::shared_ptr<const SurgSim::Math::MeshShape> mesh);

	std::shared_ptr<Localization> createNodeLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location);

	std::shared_ptr<Localization> createTriangleLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location);

	std::shared_ptr<Localization> createElementLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location);

	/// Mapping from collision triangle's id to fem element id.
	std::unordered_map<size_t, size_t> m_triangleIdToElementIdMap;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DREPRESENTATION_H
