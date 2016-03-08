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

#ifndef SURGSIM_MATH_PARTICLESSHAPE_H
#define SURGSIM_MATH_PARTICLESSHAPE_H

#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{
class AabbTree;
};

namespace Math
{

SURGSIM_STATIC_REGISTRATION(ParticlesShape);

/// Particles Shape: A shape consisting of a group of particles of equal radius
class ParticlesShape : public Shape, public SurgSim::DataStructures::Vertices<DataStructures::EmptyData>
{
public:
	/// Constructor
	/// \param radius The particles' radius (in m)
	explicit ParticlesShape(double radius = 0.0);

	/// Copy constructor
	/// \param other The ParticleShape to be copied from
	explicit ParticlesShape(const ParticlesShape& other);

	/// Copy constructor from another Vertices type
	/// \tparam V type of data stored in the other vertices
	/// \param other The vertices to be copied from. Vertex data will not be copied
	template <class V>
	explicit ParticlesShape(const SurgSim::DataStructures::Vertices<V>& other);

	/// Assignment when the template data is a different type
	/// \tparam V type of data stored in the other Vertices
	/// \param other the Vertices to copy from
	template <class V>
	ParticlesShape& operator=(const Vertices<V>& other);

	SURGSIM_CLASSNAME(SurgSim::Math::ParticlesShape);

	/// Get the AabbTree
	/// \return The object's associated AabbTree
	const std::shared_ptr<const SurgSim::DataStructures::AabbTree> getAabbTree() const;

	/// Set the particles' radius
	/// \param radius the radius being set to all particles
	void setRadius(double radius);

	/// Get the radius of the particles
	/// \return The particles' radius
	double getRadius() const;

	int getType() const override;

	double getVolume() const override;

	Vector3d getCenter() const override;

	Matrix33d getSecondMomentOfVolume() const override;

	std::shared_ptr<Shape> getTransformed(const RigidTransform3d& pose) const override;

	bool isValid() const override;

	bool isTransformable() const override;

	const Math::Aabbd& getBoundingBox() const override;

private:
	bool doUpdate() override;

	/// The aabb tree of the ParticlesShape
	std::shared_ptr<SurgSim::DataStructures::AabbTree> m_aabbTree;

	/// Particles' radius
	double m_radius;

	/// Volumetric center of particles
	Vector3d m_center;

	/// Total volume of particles
	double m_volume;

	/// Second moment of volume
	Matrix33d m_secondMomentOfVolume;
};

};
};

#include "SurgSim/Math/ParticlesShape-inl.h"

#endif // SURGSIM_MATH_PARTICLESSHAPE_H
