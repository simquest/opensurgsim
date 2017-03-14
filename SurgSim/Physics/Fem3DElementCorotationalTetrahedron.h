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
SURGSIM_STATIC_REGISTRATION(Fem3DElementCorotationalTetrahedron);

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
	Fem3DElementCorotationalTetrahedron();

	/// Constructor
	/// \param nodeIds A vector of node ids defining this tetrahedron element in a overall mesh
	/// \note It is required that the triangle ABC is CCW looking from D (i.e. dot(cross(AB, AC), AD) > 0)
	/// \note This is required from the signed volume calculation method getVolume()
	/// \note A warning will be logged when the initialize function is called if this condition is not met, but the
	/// simulation will keep running.  Behavior will be undefined because of possible negative volume terms.
	explicit Fem3DElementCorotationalTetrahedron(std::array<size_t, 4> nodeIds);

	/// Constructor for FemElement object factory
	/// \param elementData A FemElement3D struct defining this tetrahedron element in a overall mesh
	/// \note It is required that the triangle ABC is CCW looking from D (i.e. dot(cross(AB, AC), AD) > 0)
	/// \note This is required from the signed volume calculation method getVolume()
	/// \note A warning will be logged when the initialize function is called if this condition is not met, but the
	/// simulation will keep running.  Behavior will be undefined because of possible negative volume terms.
	/// \exception SurgSim::Framework::AssertionFailure if nodeIds has a size different than 4
	explicit Fem3DElementCorotationalTetrahedron(std::shared_ptr<FemElementStructs::FemElementParameter> elementData);

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem3DElementCorotationalTetrahedron);

	void initialize(const SurgSim::Math::OdeState& state) override;

	const SurgSim::Math::Matrix33d& getRotationMatrix() const;

protected:
	/// Compute the rotation, mass and stiffness matrices of the element from the given state
	/// \param state The state to compute the rotation and jacobians from
	/// \param [out] R rotation matrix of the element in the given state (can be nullptr if not needed)
	/// \param [out] Me, Ke Respectively the mass and stiffness matrices (Me and/or Ke be nullptr if not needed)
	/// \note The model is not viscoelastic but purely elastic, so there is no damping matrix here.
	void computeRotationMassAndStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix33d* R,
										 Math::Matrix* Me, Math::Matrix* Ke) const;

	void doUpdateFMDK(const Math::OdeState& state, int options) override;

	/// The constant inverse matrix of the undeformed tetrahedron homogeneous 4 points coordinates.
	/// This is useful to compute the deformation gradient from which the element rotation is extracted.
	SurgSim::Math::Matrix44d m_Vinverse;

	// The mass matrix of the linear tetrahedron
	Eigen::Matrix<double, 12, 12> m_MLinear;

	// The stiffness matrix of the linear tetrahedron
	Eigen::Matrix<double, 12, 12> m_KLinear;

	// The rotation matrix
	SurgSim::Math::Matrix33d m_R;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DELEMENTCOROTATIONALTETRAHEDRON_H
