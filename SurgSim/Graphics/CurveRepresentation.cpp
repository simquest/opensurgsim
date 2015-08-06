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

#include "SurgSim/Graphics/CurveRepresentation.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace SurgSim
{

namespace Graphics
{

CurveRepresentation::CurveRepresentation(const std::string& name) : Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CurveRepresentation, size_t, Subdivisions, getSubdivisions, setSubdivisions);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CurveRepresentation, double, Tension, getTension, setTension);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CurveRepresentation, Math::Vector4d, Color, getColor, setColor);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CurveRepresentation, double, Width, getWidth, setWidth);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CurveRepresentation, bool, AntiAliasing, isAntiAliasing, setAntiAliasing);

	// Provide a common entry point accepting VerticesPlain under the label "Vertices"
	// this can be used by behaviors to address this structure and the point cloud the same way
	auto converter = std::bind(SurgSim::Framework::convert<DataStructures::VerticesPlain>, std::placeholders::_1);
	auto functor = std::bind((void(CurveRepresentation::*)(const DataStructures::VerticesPlain&))
							 &CurveRepresentation::updateControlPoints, this, converter);

	setSetter("Vertices", functor);
}

void CurveRepresentation::updateControlPoints(const DataStructures::VerticesPlain& vertices)
{
	SURGSIM_ASSERT(vertices.getNumVertices() > 1) << "Need at least 2 control points.";
	m_locker.set(vertices);
}

void CurveRepresentation::updateControlPoints(DataStructures::VerticesPlain&& vertices)
{
	SURGSIM_ASSERT(vertices.getNumVertices() > 1) << "Need at least 2 control points.";
	m_locker.set(std::move(vertices));
}

}
}