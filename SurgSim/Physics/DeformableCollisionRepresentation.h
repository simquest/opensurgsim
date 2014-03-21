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

#include <SurgSim/Collision/Representation.h>

#include <string>
#include <memory>

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
class DeformableRepresentationState;
class DeformableRepresentationBase;

class DeformableCollisionRepresentation : public SurgSim::Collision::Representation
{
public:

	/// Constructor
	/// \param name Name of the Representation
	explicit DeformableCollisionRepresentation(const std::string& name);

	/// Destructor
	virtual ~DeformableCollisionRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::DeformableCollisionRepresentation);

	/// \return The mesh that is part of this representation
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> getMesh() const;

	/// Overridden from Collision::Representation
	/// This will fail if the shape is not a mesh shape
	void setShape(std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Overridden from Collision::Representation
	const std::shared_ptr<SurgSim::Math::Shape> getShape() const;

	///
	void setMesh(std::shared_ptr<SurgSim::DataStructures::TriangleMesh> mesh);

	/// Sets the deformable to which this collision representation is connected, this will call
	/// setCollisionRepresentation in the deformable
	/// \param representation The deformable that will be used to update the contained mesh
	void setDeformableRepresentation(std::shared_ptr<SurgSim::Physics::DeformableRepresentationBase> representation);

	const std::shared_ptr<SurgSim::Physics::DeformableRepresentationBase> getDeformable() const;

	/// Overridden from Collision::Representation
	virtual int getShapeType() const override;

	/// Overridden from Collision::Representation
	virtual void update(const double& dt) override;

	/// Overridden from Collision::Representation
	virtual bool doInitialize() override;

	/// Overridden from Collision::Representation
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Overridden from Collision::Representation
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const override;

	/// Overridden from Collision::Representation
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Overridden from Collision::Representation
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;

private:

	std::shared_ptr<SurgSim::Math::MeshShape> m_shape;
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> m_mesh;
	std::weak_ptr<SurgSim::Physics::DeformableRepresentationBase> m_deformable;
};

}
}

#endif
