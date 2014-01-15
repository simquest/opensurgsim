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

#ifndef SURGSIM_PHYSICS_FEMREPRESENTATION_H
#define SURGSIM_PHYSICS_FEMREPRESENTATION_H

#include <memory>

#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/FemElement.h"

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

/// Finite Element Model (a.k.a. fem) is a deformable model (a set of nodes connected by FemElement).
/// \note A fem is a DeformableRepresentation (Physics::Representation and Math::OdeEquation)
/// \note Therefore, it defines a dynamic system M.a=F(x,v)
/// \note The model handles damping through the Rayleigh damping (where damping is a combination of mass and stiffness)
/// \note At this level, no assumption is made on the matrices type. The derived classes will specialize the type.
/// \tparam MT Mass matrix type
/// \tparam DT Damping matrix type
/// \tparam KT Stiffness matrix type
/// \tparam ST System matrix type (best type to store a combination of matrices of type MT, DT and KT)
template <class MT, class DT, class KT, class ST>
class FemRepresentation: public DeformableRepresentation<MT, DT, KT, ST>
{
public:
	/// Constructor
	/// \param name The name of the FemRepresentation
	explicit FemRepresentation(const std::string& name);

	/// Destructor
	virtual ~FemRepresentation();

	/// Adds a FemElement
	/// \param element The FemElement to add to the representation
	void addFemElement(const std::shared_ptr<FemElement> element);

	/// Gets the number of FemElement
	/// \return the number of FemElement
	unsigned int getNumFemElements() const;

	/// Retrieves a given FemElement from its id
	/// \param femElementId The FemElement id for which the FemElement is requested
	/// \return The FemElement for the given femElementId
	/// \note The FemElement is returned with read/write access
	/// \note Out of range femElementId will raise an exception
	std::shared_ptr<FemElement> getFemElement(unsigned int femElementId);

	/// Gets the total mass of the fem
	/// \return The total mass of the fem (in Kg)
	double getTotalMass() const;

	/// Gets the Rayleigh stiffness parameter
	/// \return The Rayleigh stiffness parameter
	double getRayleighDampingStiffness() const;

	/// Gets the Rayleigh mass parameter
	/// \return The Rayleigh mass parameter
	double getRayleighDampingMass() const;

	/// Sets the Rayleigh stiffness parameter
	/// \param stiffnessCoef The Rayleigh stiffness parameter
	void setRayleighDampingStiffness(double stiffnessCoef);

	/// Sets the Rayleigh mass parameter
	/// \param massCoef The Rayleigh mass parameter
	void setRayleighDampingMass(double massCoef);

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt) override;

	/// Updates the representation state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt) override;

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt) override;

	/// Update the Representation's current position and velocity using a time interval, dt, and change in velocity,
	/// deltaVelocity.
	///
	/// This function typically is called in the physics pipeline (PhysicsManager::doUpdate) after solving the equations
	/// that enforce constraints when collisions occur.  Specifically it is called in the PushResults::doUpdate step.
	/// \param dt The time step
	/// \param deltaVelocity The block of a vector containing the correction to be applied to the dof
	virtual void applyCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity) override;

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	virtual SurgSim::Math::Vector& computeF(const DeformableRepresentationState& state) override;

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	virtual const MT& computeM(const DeformableRepresentationState& state) override;

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	virtual const DT& computeD(const DeformableRepresentationState& state) override;

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	virtual const KT& computeK(const DeformableRepresentationState& state) override;

	/// Evaluation of f(x,v), M(x,v), D = -df/dv(x,v), K = -df/dx(x,v)
	/// When all the terms are needed, this method can perform optimization in evaluating everything together
	/// \param state (x, v) the current position and velocity to evaluate the various terms with
	/// \param[out] f The RHS f(x,v)
	/// \param[out] M The matrix M(x,v)
	/// \param[out] D The matrix D = -df/dv(x,v)
	/// \param[out] K The matrix K = -df/dx(x,v)
	/// \note Returns pointers, the internal data will remain unchanged until the next call to computeFMDK() or
	/// \note computeF(), computeM(), computeD(), computeK()
	virtual void computeFMDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector** f, MT** M, DT** D, KT** K) override;

protected:
	/// Adds the Rayleigh damping forces
	/// \param[in,out] f The force vector to cumulate the Rayleigh damping force into
	/// \param state The state vector containing positions and velocities
	/// \param useGlobalDampingMatrix True indicates that the global stiffness matrix D should be used (F = -D.v)
	/// \param useGlobalMassMatrix, useGlobalStiffnessMatrix True indicates that the global mass and stiffness matrices
	///        should be used (F = -c.M.v - d.K.v)
	/// \param scale A scaling factor to apply on the damping force
	/// \note Damping matrix D = c.M + d.K (Rayleigh damping definition)
	/// \note F = - D.v
	/// \note F = - (c.M.v + d.K.v)
	/// \note If useGlobalDampingMatrix is True, D will be used
	/// \note Otherwise, if {useGlobalMassMatrix | useGlobalStiffnessMatrix} is True, {M | K} will be used instead.
	/// \note If useGlobalDampingMatrix is False and useGlobalMassMatrix      is False
	/// \note    the mass      component will be computed FemElement by FemElement
	/// \note If useGlobalDampingMatrix is False and useGlobalStiffnessMatrix is False
	/// \note    the stiffness component will be computed FemElement by FemElement
	void addRayleighDampingForce(SurgSim::Math::Vector* f, const DeformableRepresentationState& state,
		bool useGlobalDampingMatrix = false,
		bool useGlobalMassMatrix = false, bool useGlobalStiffnessMatrix = false,
		double scale = 1.0);

	/// Adds the FemElements forces to f (given a state)
	/// \param[in,out] f The force vector to cumulate the FemElements forces into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the FemElements forces with
	void addFemElementsForce(SurgSim::Math::Vector* f, const DeformableRepresentationState& state, double scale = 1.0);

	/// Adds the gravity force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the gravity force into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the gravity force with
	/// \note This method does not do anything if gravity is disabled
	void addGravityForce(SurgSim::Math::Vector *f, const DeformableRepresentationState& state, double scale = 1.0);

private:
	/// Interface to be implemented by derived classes
	/// \return True if component is initialized successfully; otherwise, false.
	virtual bool doInitialize() override;

	/// FemElements
	std::vector<std::shared_ptr<FemElement>> m_femElements;

	/// Useful information per node
	std::vector<double> m_massPerNode; //< Useful in seting up the gravity force F=mg

	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, M = mass, K = stiffness
	struct {
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;

	// Dependent names resolution (need to be in public/protected to be accessible in derived classes)
public:
	// Used API from Physics::OdeEquation through Physics::DeformableRepresentation
	using DeformableRepresentation<MT, DT, KT, ST>::m_initialState;

	// Used API from Physics::DeformableRepresentation
	using DeformableRepresentation<MT, DT, KT, ST>::m_previousState;
	using DeformableRepresentation<MT, DT, KT, ST>::m_currentState;
	using DeformableRepresentation<MT, DT, KT, ST>::m_newState;
	using DeformableRepresentation<MT, DT, KT, ST>::m_finalState;
	using DeformableRepresentation<MT, DT, KT, ST>::getNumDofPerNode;
	using DeformableRepresentation<MT, DT, KT, ST>::m_odeSolver;
	using DeformableRepresentation<MT, DT, KT, ST>::m_f;
	using DeformableRepresentation<MT, DT, KT, ST>::m_M;
	using DeformableRepresentation<MT, DT, KT, ST>::m_D;
	using DeformableRepresentation<MT, DT, KT, ST>::m_K;

	// Used API from Physics::Representation through Physics::DeformableRepresentation
	using DeformableRepresentation<MT, DT, KT, ST>::isActive;
	using DeformableRepresentation<MT, DT, KT, ST>::isGravityEnabled;
	using DeformableRepresentation<MT, DT, KT, ST>::getNumDof;
	using DeformableRepresentation<MT, DT, KT, ST>::getGravity;
};

} // namespace Physics

} // namespace SurgSim

#include "SurgSim/Physics/FemRepresentation-inl.h"

#endif // SURGSIM_PHYSICS_FEMREPRESENTATION_H
