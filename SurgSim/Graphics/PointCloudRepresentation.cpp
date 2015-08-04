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

#include <memory>

#include "SurgSim/Graphics/PointCloudRepresentation.h"

namespace SurgSim
{
namespace Graphics
{

PointCloudRepresentation::PointCloudRepresentation(const std::string& name) : Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PointCloudRepresentation, double, PointSize, getPointSize, setPointSize);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PointCloudRepresentation, SurgSim::Math::Vector4d, Color, getColor, setColor);
}

PointCloudRepresentation::~PointCloudRepresentation()
{
}

void PointCloudRepresentation::updateVertices(const DataStructures::VerticesPlain& vertices)
{
	m_locker.set(vertices);
}

void PointCloudRepresentation::updateVertices(DataStructures::VerticesPlain&& vertices)
{
	m_locker.set(std::move(vertices));
}

}; // Graphics
}; // SurgSim
