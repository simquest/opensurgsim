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

#ifndef SURGSIM_PHYSICS_FEM3DELEMENTCOROTATIONALTETRAHEDRON_H
#define SURGSIM_PHYSICS_FEM3DELEMENTCOROTATIONALTETRAHEDRON_H

#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"

namespace SurgSim
{

namespace Physics
{

/// Fem Element 3D co-rotational based on a tetrahedron volume discretization
/// \note This class derives from the linear version of the FEM 3D element tetrahedron, adding a rigid frame
/// \note attached to the tetrahedron element to follow its rigid motion. Therefore it computes deformation in the
/// \note local coordinate system of the tetrahedron. This class is based on 2 papers:
/// \note "Interactive Virtual Materials", Muller, Gross. Graphics Interface 2004
/// \note "Exact Corotational Linear FEM Stiffness Matrix", Jernej Barbic. Technical Report USC 2012.
/// \note Only the force and stiffness calculation are different, involving some changes as well in addMatVec
/// \note (it uses the updated stiffness matrix).
/// \note The update method takes care of extracting the rigid motion of the element.
/// \note This element is updating its stiffness matrix at each new time step, which means that it cannot
/// \note be used with any OdeSolverLinearXXX, it needs an ode solver that recomputes the data at each iteration.
class Fem3DElementCorotationalTetrahedron : public Fem3DElementTetrahedron
{
public:
	/// Constructor
	/// \param nodeIds An array of 4 node (A, B, C, D) ids defining this tetrahedron element in a overall mesh
	/// \note It is required that the triangle ABC is CCW looking from D (i.e. dot(cross(AB, AC), AD) > 0)
	/// \note This is required from the signed volume calculation method getVolume()
	/// \note A warning will be logged when the initialize function is called if this condition is not met, but the
	/// simulation will keep running.  Behavior will be undefined because of possible negative volume terms.
	explicit Fem3DElementCorotationalTetrahedron(std::array<size_t, 4> nodeIds);

	void initialize(const SurgSim::Math::OdeState& state) override;

	void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale = 1.0) override;

	void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K, double scale = 1.0) override;

	void addMatVec(const SurgSim::Math::OdeState& state,
		double alphaM, double alphaD, double alphaK,
		const SurgSim::Math::Vector& vector, SurgSim::Math::Vector* result) override;

	/// Update the element co-rotational frame. Updating as well the stiffness matrix.
	/// \param state The state from which the element rigid transformation needs to be computed
	/// \return True if the update was successful, false otherwise, in which case the representation should be
	/// deactivated (invalid data).
	bool update(const SurgSim::Math::OdeState& state) override;

protected:
	/// The element rigid rotation
	SurgSim::Math::Matrix33d m_rotation;

	/// The co-rotational stiffness matrix
	Eigen::Matrix<double, 12, 12> m_corotationalStiffnessMatrix;

	/// The constant inverse matrix of the undeformed tetrahedron homogeneous 4 points coordinates.
	/// This is useful to compute the deformation gradient from which the element rotation is extracted.
	SurgSim::Math::Matrix44d m_Vinverse;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DELEMENTCOROTATIONALTETRAHEDRON_H
