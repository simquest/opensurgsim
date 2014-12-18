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

#ifndef SURGSIM_PHYSICS_DEFORMABLECOLLISIONREPRESENTATION_H
#define SURGSIM_PHYSICS_DEFORMABLECOLLISIONREPRESENTATION_H

#include <memory>
#include <string>

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{
namespace DataStructures
{
class TriangleMesh;
}

namespace Math
{
class Shape;
class MeshShape;
}

namespace Physics
{
class DeformableRepresentation;

SURGSIM_STATIC_REGISTRATION(DeformableCollisionRepresentation);

/// A collision representation that can be attached to a deformable, when this contains a mesh with the same number
/// of vertices as the deformable has nodes, the mesh vertices will move to match the positions of the nodes in
/// the deformable.
class DeformableCollisionRepresentation : public SurgSim::Collision::Representation
{
public:

	/// Constructor
	/// \param name Name of the Representation
	explicit DeformableCollisionRepresentation(const std::string& name);

	/// Destructor
	virtual ~DeformableCollisionRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::DeformableCollisionRepresentation);

	/// Set the mesh to be used in this collision representation
	/// the vertices in the mesh need to be the same number as the vertices in the deformable representation.
	/// \param mesh The mesh to be used for the collision calculation and updates
	/// \note The shape held by this deformable collision representation will be updated as well.
	void setMesh(std::shared_ptr<SurgSim::DataStructures::TriangleMesh> mesh);

	/// \return The mesh that is part of this representation
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> getMesh() const;

	/// Set the shape for this collision representation, has to be a SurgSim::Math::MeshShape.
	/// \param shape The shape to be used.
	/// \note The mesh held by this deformable collision representation will be updated as well.
	void setShape(std::shared_ptr<SurgSim::Math::Shape> shape);

	const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;

	/// Sets the deformable to which this collision representation is connected
	/// \param representation The deformable that will be used to update the contained mesh
	void setDeformableRepresentation(std::shared_ptr<SurgSim::Physics::DeformableRepresentation> representation);

	/// \return The deformable that is used to update the contained mesh
	const std::shared_ptr<SurgSim::Physics::DeformableRepresentation> getDeformableRepresentation() const;

	int getShapeType() const override;

	void update(const double& dt) override;

private:
	bool doInitialize() override;
	bool doWakeUp() override;

	/// Shape used for collision detection
	std::shared_ptr<SurgSim::Math::MeshShape> m_shape;

	/// Mesh used for collision detection
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> m_mesh;

	/// Reference to the deformable driving changes to this mesh
	std::weak_ptr<SurgSim::Physics::DeformableRepresentation> m_deformable;
};

} // namespace Physics
} // namespace SurgSim

#endif
