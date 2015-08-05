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

#ifndef SURGSIM_GRAPHICS_CURVEREPRESENTATION_H
#define SURGSIM_GRAPHICS_CURVEREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/DataStructures/EmptyData.h"

namespace SurgSim
{

namespace Graphics
{

class CurveRepresentation : public virtual Representation

{
public:
	/// Constructor
	explicit CurveRepresentation(const std::string& name);

	typedef DataStructures::Vertices<DataStructures::EmptyData> ControlPointType;

	void updateControlPoints(const ControlPointType& vertices);

	void updateControlPoints(ControlPointType&& vertices);

protected:

	Framework::LockedContainer<ControlPointType> m_locker;

};

}
}

#endif // SURGSIM_GRAPHICS_CURVEREPRESENTATION_H
