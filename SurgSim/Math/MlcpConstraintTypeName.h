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

#ifndef SURGSIM_MATH_MLCPCONSTRAINTTYPENAME_H
#define SURGSIM_MATH_MLCPCONSTRAINTTYPENAME_H

#include <string>
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Math
{

inline std::string getMlcpConstraintTypeName(MlcpConstraintType constraintType)
{
	switch (constraintType)
	{
	case MLCP_BILATERAL_1D_CONSTRAINT:
		return "MLCP_BILATERAL_1D_CONSTRAINT";
	case MLCP_BILATERAL_2D_CONSTRAINT:
		return "MLCP_BILATERAL_2D_CONSTRAINT";
	case MLCP_BILATERAL_3D_CONSTRAINT:
		return "MLCP_BILATERAL_3D_CONSTRAINT";
	case MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT:
		return "MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT";
//	case MLCP_BILATERAL_4D_CONSTRAINT:
//		return "MLCP_BILATERAL_4D_CONSTRAINT";
	case MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
		return "MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT";
	case MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT:
		return "MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT";
	case MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT:
		return "MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT";
	case MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT:
		return "MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT";
	default:
		SURGSIM_LOG_SEVERE(SURGSIM_ASSERT_LOGGER) << "bad MLCP constraint type value: " << constraintType;
		return "";
	}
}

inline MlcpConstraintType getMlcpConstraintTypeValue(const std::string& typeName)
{

	if (typeName == "MLCP_BILATERAL_1D_CONSTRAINT")
	{
		return MLCP_BILATERAL_1D_CONSTRAINT;
	}
	else if (typeName == "MLCP_BILATERAL_2D_CONSTRAINT")
	{
		return MLCP_BILATERAL_2D_CONSTRAINT;
	}
	else if (typeName == "MLCP_BILATERAL_3D_CONSTRAINT")
	{
		return MLCP_BILATERAL_3D_CONSTRAINT;
	}
	else if (typeName == "MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT")
	{
		return MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT;
	}
// 	else if (typeName == "MLCP_BILATERAL_4D_CONSTRAINT")
// 	{
// 		return MLCP_BILATERAL_4D_CONSTRAINT;
// 	}
	else if (typeName == "MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT")
	{
		return MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
	}
	else if (typeName == "MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT")
	{
		return MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT;
	}
	else if (typeName == "MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT")
	{
		return MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT;
	}
	else if (typeName == "MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT")
	{
		return MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT;
	}
	else
	{
		SURGSIM_LOG_WARNING(SURGSIM_ASSERT_LOGGER) << "bad MLCP constraint type name: '" << typeName << "'";
		return MLCP_INVALID_CONSTRAINT;
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPCONSTRAINTTYPENAME_H
