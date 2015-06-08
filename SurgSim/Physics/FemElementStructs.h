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

#ifndef SURGSIM_PHYSICS_FEMELEMENTSTRUCTS_H
#define SURGSIM_PHYSICS_FEMELEMENTSTRUCTS_H

namespace SurgSim
{
namespace Physics
{
namespace FemElementStructs
{
struct RotationVectorData
{
	RotationVectorData()
	{
		thetaX = 0.0;
		thetaY = 0.0;
		thetaZ = 0.0;
	}

	bool operator==(const RotationVectorData& rhs) const
	{
		return (thetaX == rhs.thetaX && thetaY == rhs.thetaY && thetaZ == rhs.thetaZ);
	}
	double thetaX;
	double thetaY;
	double thetaZ;
};

struct FemElementParameter
{
	FemElementParameter()
	{
		youngModulus = 0.0;
		poissonRatio = 0.0;
		massDensity = 0.0;
	}

	virtual ~FemElementParameter(){}

	std::string type;   // “LinearBeam”, “CorotationalTetrahedron”…

	double youngModulus;
	double poissonRatio;
	double massDensity;
};

struct FemElement1DParameter : public FemElementParameter
{
	FemElement1DParameter()
	{
		FemElementParameter();
		radius = 0.0;
		enableShear = false;
	}

	double radius;
	bool enableShear;
};

struct FemElement2DParameter : public FemElementParameter
{
	FemElement2DParameter()
	{
		FemElementParameter();
		thickness = 0.0;
	}

	double thickness;
};

struct FemElement3DParameter : public FemElementParameter {};
} // namespace FemElementStructs
} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENTSTRUCTS_H
