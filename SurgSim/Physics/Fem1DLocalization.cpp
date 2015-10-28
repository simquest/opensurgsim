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

#include "SurgSim/Physics/Fem1DLocalization.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

Fem1DLocalization::Fem1DLocalization(
	std::shared_ptr<Representation> representation,
	const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition) :
	FemLocalization(representation, localPosition)
{
}

Fem1DLocalization::~Fem1DLocalization()
{
}

bool Fem1DLocalization::isValidRepresentation(std::shared_ptr<Representation> representation)
{
	auto femRepresentation = std::dynamic_pointer_cast<Fem1DRepresentation>(representation);

	// Allows to reset the representation to nullptr ...
	return (femRepresentation != nullptr || representation == nullptr);
}

} // namespace Physics

} // namespace SurgSim
