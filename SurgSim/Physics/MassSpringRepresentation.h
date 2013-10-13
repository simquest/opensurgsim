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

#ifndef SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H
#define SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H

#include <memory>

#include <SurgSim/Physics/DeformableRepresentation.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>
#include <SurgSim/Physics/Mass.h>
#include <SurgSim/Physics/Spring.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/OdeEquation.h>
#include <SurgSim/Math/OdeSolver.h>

using SurgSim::Math::OdeEquation;
using SurgSim::Math::OdeSolver;
using SurgSim::Math::DiagonalMatrix;
using SurgSim::Math::Matrix;
using SurgSim::Math::Vector;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{

namespace Physics
{

/// MassSpring model is a deformable model (a set of masses connected by springs).
/// \note The class can handle 3 type of numerical integration scheme (Euler explicit, modified and implicit).
/// \note The model handles damping through the Rayleigh damping (damping is a combination of mass and stiffness).
/// \note It is a DeformableRepresentation
/// \note It is a OdeEquation (defines a dynamic system M.a=F(x,v)) with M diagonal
class MassSpringRepresentation:
	public DeformableRepresentation<DiagonalMatrix, Matrix, Matrix>,
	public OdeEquation<DeformableRepresentationState, DiagonalMatrix, Matrix, Matrix, Matrix>
{
public:
	/// The diverse numerical integration scheme supported
	enum IntegrationScheme {
		INTEGRATIONSCHEME_EXPLICIT_EULER = 0,
		INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
		INTEGRATIONSCHEME_IMPLICIT_EULER
	};

	/// Constructor
	/// \param name The name of the MassSpringRepresentation
	explicit MassSpringRepresentation(const std::string& name);

	/// Destructor
	virtual ~MassSpringRepresentation();

	/// Adds a mass
	/// \param mass The mass to add to the representation
	/// \note Masses are kept in an ordered list, giving them an index
	/// \note This mass will be associated with the node of same index
	void addMass(const std::shared_ptr<Mass> mass);

	/// Adds a spring
	/// \param spring The spring to add to the representation
	void addSpring(const std::shared_ptr<Spring> spring);

	/// Gets the number of masses
	/// \return the number of masses
	unsigned int getNumMasses() const;

	/// Gets the number of springs
	/// \return the number of springs
	unsigned int getNumSprings() const;

	/// Retrieves the mass of a given node
	/// \param nodeId The node id for which the mass is requested
	/// \return the mass attribute of a node
	const std::shared_ptr<Mass> getMass(unsigned int nodeId) const;
	
	/// Retrieves a given spring from its id
	/// \param springId The spring id for which the spring is requested
	/// \return the spring for the given springId
	const std::shared_ptr<Spring> getSpring(unsigned int springId) const;

	/// Gets the total mass of the mass spring
	/// \return The total mass of the mass spring (in Kg)
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

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	void setIntegrationScheme(IntegrationScheme integrationScheme);
	
	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	IntegrationScheme getIntegrationScheme() const;

	/// Query the representation type
	/// \return the RepresentationType for this representation
	virtual RepresentationType getType() const override;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt) override;

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt) override;

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt) override;

	/// Apply a correction to the internal degrees of freedom
	/// \param dt The time step
	/// \param block The block of a vector containing the correction to be applied to the dof
	virtual void applyDofCorrection(double dt, const Eigen::VectorBlock<Vector>& block) override;

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	virtual Vector& computeF(const DeformableRepresentationState& state) override;

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	virtual const DiagonalMatrix& computeM(const DeformableRepresentationState& state) override;

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	virtual const Matrix& computeD(const DeformableRepresentationState& state) override;

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	virtual const Matrix& computeK(const DeformableRepresentationState& state) override;

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
		Vector** f, DiagonalMatrix** Mass, Matrix** Damping, Matrix** Stiffness) override;

protected:
	/// Add the Rayleigh damping forces
	/// \param[in,out] f The force vector to cumulate the Rayleigh damping force into
	/// \param state The state vector containing positions and velocities
	/// \param useGlobalStiffnessMatrix True is the global stiffness matrix should be used, False otherwise
	/// \param scale A scaling factor to apply on the damping force
	/// \note M.a + D.v + K.x = F          with D = c.M + d.K (Rayleigh damping definition)
	/// \note M.a + K.x = F - (c.M.v + d.K.v)
	void addRayleighDampingForce(Vector* f, const DeformableRepresentationState& state,
		bool useGlobalStiffnessMatrix = false, double scale = 1.0);

	/// Add the springs force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the spring forces into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the spring forces with
	void addSpringsForce(Vector* f, const DeformableRepresentationState& state, double scale = 1.0);

	/// Add the gravity force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the gravity force into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the gravity force with
	/// \note This method does not do anything if gravity is disabled
	void addGravityForce(Vector *f, const DeformableRepresentationState& state, double scale = 1.0);

	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	void transformState(std::shared_ptr<DeformableRepresentationState> state, const RigidTransform3d& transform);

private:
	/// Masses
	std::vector<std::shared_ptr<Mass>> m_masses;
	
	/// Springs
	std::vector<std::shared_ptr<Spring>> m_springs;

	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, M = mass, K = stiffness
	struct {
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	IntegrationScheme m_integrationScheme;

	/// Ode solver (its type depends on the numerical integration scheme)
	std::shared_ptr<OdeSolver<DeformableRepresentationState, DiagonalMatrix, Matrix, Matrix, Matrix>> m_odeSolver;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H
