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

#ifndef SURGSIM_BLOCKS_MASSSPRING3DREPRESENTATION_H
#define SURGSIM_BLOCKS_MASSSPRING3DREPRESENTATION_H

#include <array>
#include <vector>

#include "SurgSim/Physics/MassSpringRepresentation.h"

namespace SurgSim
{

namespace Blocks
{

// This class defines a simple MassSpring 3D structures
class MassSpring3DRepresentation : public SurgSim::Physics::MassSpringRepresentation
{
public:
	/// Constructor
	/// \param name The model name
	explicit MassSpring3DRepresentation(const std::string& name) :
		SurgSim::Physics::MassSpringRepresentation(name)
	{
	}

	/// Initializes a 3D model
	/// \param extremities 8 positions forming the extremities of the 3D regular model (8 corners)
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 3)
	/// \param nodeBoundaryConditions The list of all nodeId being boundary conditions (fixed node)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param stiffnessStretching, dampingStretching The spring param for all stretching springs (edges)
	/// \param stiffnessBending, dampingBending The spring param for all bending springs (edges)
	/// \param stiffnessFaceDiagonal, dampingFaceDiagonal The spring param for all face diagonal springs (faces)
	/// \param stiffnessVolumeDiagonal, dampingVolumeDiagonal The spring param for all volume diagonal springs (volume)
	/// \note Stretching springs are connecting neighbors, bending springs are connecting 1 node
	/// \note to its 2nd degree neighbors, creating a bending force around the middle node.
	/// \note Face diagonal springs aim at maintaining the area of a square
	/// \note Volume diagonal springs aim at maintaining the volume of a cube
	/// \note extremities are organized as follow:
	/// \note          [0][1][0] *---* [1][1][0]
	/// \note        [0][1][1] *---* [1][1][1]
	/// \note   [0][0][0] <-   |   | *         -> [1][0][0]
	/// \note        [0][0][1] *---* [1][0][1]
	void init3D(
		const std::array<std::array<std::array<SurgSim::Math::Vector3d, 2>, 2>, 2> extremities,
		size_t numNodesPerDim[3],
		std::vector<size_t> nodeBoundaryConditions,
		double totalMass,
		double stiffnessStretching, double dampingStretching,
		double stiffnessBending, double dampingBending,
		double stiffnessFaceDiagonal, double dampingFaceDiagonal,
		double stiffnessVolumeDiagonal, double dampingVolumeDiagonal);

private:
	/// Helper method to initialize/add all stretching springs on a 3D structure
	/// \param mesh The MassSpring to initialize the springs with (rest lengths calculation)
	/// \param numNodesPerDim The number of nodes on the 3 dimensions
	/// \param stiffness, damping The spring parameters
	void init3DStretchingSprings(const std::shared_ptr<Physics::MassSpring> mesh,
		size_t numNodesPerDim[3], double stiffness, double damping);
	/// Helper method to initialize/add all bending springs on a 3D structure
	/// \param mesh The MassSpring to initialize the springs with (rest lengths calculation)
	/// \param numNodesPerDim The number of nodes on the 3 dimensions
	/// \param stiffness, damping The spring parameters
	void init3DBendingSprings(const std::shared_ptr<Physics::MassSpring> mesh,
		size_t numNodesPerDim[3], double stiffness, double damping);
	/// Helper method to initialize/add all face diagonal springs on a 3D structure
	/// \param mesh The MassSpring to initialize the springs with (rest lengths calculation)
	/// \param numNodesPerDim The number of nodes on the 3 dimensions
	/// \param stiffness, damping The spring parameters
	void init3DFaceDiagonalSprings(const std::shared_ptr<Physics::MassSpring> mesh,
		size_t numNodesPerDim[3], double stiffness, double damping);
	/// Helper method to initialize/add all volume diagonal springs on a 3D structure
	/// \param mesh The MassSpring to initialize the springs with (rest lengths calculation)
	/// \param numNodesPerDim The number of nodes on the 3 dimensions
	/// \param stiffness, damping The spring parameters
	void init3DVolumeDiagonalSprings(const std::shared_ptr<Physics::MassSpring> mesh,
		size_t numNodesPerDim[3], double stiffness, double damping);
};

};  // namespace Blocks

};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_MASSSPRING3DREPRESENTATION_H
