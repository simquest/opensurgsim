// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_CONSTRAINTTYPE_H
#define SURGSIM_PHYSICS_CONSTRAINTTYPE_H

namespace SurgSim
{
namespace Physics
{

enum ConstraintType
{
	INVALID_CONSTRAINT = -1,
	FIXED_3DPOINT = 0,
	FIXED_3DROTATION_VECTOR,
	FRICTIONLESS_3DCONTACT,
	FRICTIONAL_3DCONTACT,
	FRICTIONLESS_SLIDING,
	FRICTIONAL_SLIDING,
	NUM_CONSTRAINT_TYPES
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_CONSTRAINTTYPE_H
