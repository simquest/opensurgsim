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

#ifndef SURGSIM_MATH_MLCPCONSTRAINTTYPE_H
#define SURGSIM_MATH_MLCPCONSTRAINTTYPE_H

namespace SurgSim
{
namespace Math
{

enum MlcpConstraintType
{
	// Invalid value -- not used for any constraint type
	MLCP_INVALID_CONSTRAINT = -1,
	// Fixing 1 DOF
	MLCP_BILATERAL_1D_CONSTRAINT = 0,
	// Fixing 2 DOF
	MLCP_BILATERAL_2D_CONSTRAINT,
	// Fixing 3 DOF (could be a fixed point in 3D space for example)
	MLCP_BILATERAL_3D_CONSTRAINT,
	// Fixing 3 DOF (Rotation vector)
	MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT,
	// Fixing 4 DOF (could be a fixed point with twist included for the MechanicalSpline for example)
	//	MLCP_BILATERAL_4D_CONSTRAINT,
	// Frictionless contact (could be in 2D or 3D => only 1 atomic unilateral constraint)
	MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT,
	// Frictional contact in 3D, using Coulomb friction => 1 entry in the friction coefficient array !
	MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT,
	// Frictionless suturing in 3D => 2 directional constraint = 2 bilateral constraints !
	MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT,
	// Frictional suturing in 3D => 2 directional constraint + 1 frictional tangent constraint linked by Coulomb's law
	MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT,
	MLCP_NUM_CONSTRAINT_TYPES
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPCONSTRAINTTYPE_H
