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

/*
#include <gtest/gtest.h>

#include <unordered_map>
#include <memory>
*/

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

class DivisibleCubeRepresentation : public Fem3DRepresentation
{
public:
	/// Constructor
	/// \param name	The name of the divisible cube representation.
	/// \param nodesPerAxis	The number of nodes per axis
	DivisibleCubeRepresentation(const std::string& name, size_t nodesPerAxis);


	/// Initialize with setting up the complete system so that we can separately
	/// time the component algorithms.
	/// \return Initialization success or failure
	bool noSetInitialize();

	/// Return a pointer to the OdeSolver component
	/// \return The ODE solver
	std::shared_ptr<SurgSim::Math::OdeSolver> getOdeSolver();

protected:
	/// Convert a node index from a 3d indexing to a 1d indexing
	/// \param i, j, k Indices along the X, Y and Z axis
	/// \return Unique index of the corresponding point (to access a linear array for example)
	size_t get1DIndexFrom3D(size_t i, size_t j, size_t k);

	/// Fills up a given state with the cube's nodes, border nodes, and internal nodes
	/// \param[in,out] state	The state to be filled up
	void fillUpDeformableState(std::shared_ptr<SurgSim::Math::OdeState> state);

	/// Adds the Fem3D elements of small cubes
	/// \param state	The state for initialization.
	void addFemCubes(std::shared_ptr<SurgSim::Math::OdeState> state);

private:
	// Number of point per dimensions
	size_t m_numNodesPerAxis;

	// Corner nodes of the original cube
	std::array<SurgSim::Math::Vector3d, 8> m_cubeNodes;
};

} // namespace Physics
} // namespace SurgSim
