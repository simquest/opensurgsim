// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_MATH_VERTICESSHAPE_H
#define SURGSIM_MATH_VERTICESSHAPE_H

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace Math
{

/// A Shape that also inherits from Vertices is transformable and carries a member variable of the initial Vertices for
/// transforming.
class VerticesShape : public Shape
{
public:
	bool isTransformable() const override;

	/// Set the initial Vertices.
	/// \param vertices The initial vertices.
	void setInitialVertices(const DataStructures::Vertices<DataStructures::EmptyData>& vertices);

	/// Set the initial Vertices via r-value.
	/// \param vertices The initial vertices.
	void setInitialVertices(DataStructures::Vertices<DataStructures::EmptyData>&& vertices);

	/// Get the initial Vertices.
	/// \return The initial Vertices.
	const DataStructures::Vertices<DataStructures::EmptyData>& getInitialVertices() const;

protected:
	/// The initial vertex positions.
	DataStructures::Vertices<DataStructures::EmptyData> m_initialVertices;
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_VERTICESSHAPE_H
