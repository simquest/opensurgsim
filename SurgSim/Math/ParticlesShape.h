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

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace DataStructures
{
class AabbTree;
};

namespace Math
{

class ParticlesShape : public Shape, public SurgSim::DataStructures::Vertices<SurgSim::DataStructures::EmptyData>
{
public:
	/// Constructor
	ParticlesShape();

	/// Copy constructor when the template data is a different type
	/// \tparam	VertexData Type of extra data stored in each vertex
	/// \param other The vertices to be copied from. Vertex data will not be copied
	template <class VertexData>
	explicit ParticlesShape(const SurgSim::DataStructures::Vertices<VertexData>& other)
	{
		getVertices().reserve(other.getVertices().size());
		for (auto& otherVertex : other.getVertices())
		{
			addVertex(VertexType(otherVertex));
		}
	}

	int getType() const override;
	double getVolume() const override;
	Vector3d getCenter() const override;
	Matrix33d getSecondMomentOfVolume() const override;
	bool isValid() const override;

	/// Get the AabbTree
	/// \return The object's associated AabbTree
	const std::shared_ptr<const SurgSim::DataStructures::AabbTree> getAabbTree() const;

	double getRadius() const;
	void setRadius(double radius);

private:
	void doUpdate() override;

	std::shared_ptr<SurgSim::DataStructures::AabbTree> m_aabbTree;
	double m_radius;
};

};
};

#endif // SURGSIM_MATH_PARTICLESSHAPE_H
