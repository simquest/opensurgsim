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

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace SurgSim
{

namespace Physics
{

class FemElement;
class FemPlyReaderDelegate;

/// Finite Element Model (a.k.a. fem) is a deformable model (a set of nodes connected by FemElement).
/// \note A fem is a DeformableRepresentation (Physics::Representation and Math::OdeEquation)
/// \note Therefore, it defines a dynamic system M.a=F(x,v)
/// \note The model handles damping through the Rayleigh damping (where damping is a combination of mass and stiffness)
class FemRepresentation : public DeformableRepresentation
{
public:
	/// Constructor
	/// \param name The name of the FemRepresentation
	explicit FemRepresentation(const std::string& name);

	/// Destructor
	virtual ~FemRepresentation();

	/// Sets the name of the file to be loaded
	/// \param filename The name of the file to be loaded
	void setFilename(const std::string& filename);

	/// Gets the name of the file to be loaded
	/// \return filename The name of the file to be loaded
	const std::string& getFilename() const;

	/// Loads the file
	/// \return true if successful
	bool loadFile();

	/// Adds a FemElement
	/// \param element The FemElement to add to the representation
	void addFemElement(const std::shared_ptr<FemElement> element);

	/// Gets the number of FemElement
	/// \return the number of FemElement
	size_t getNumFemElements() const;

	/// Retrieves a given FemElement from its id
	/// \param femElementId The FemElement id for which the FemElement is requested
	/// \return The FemElement for the given femElementId
	/// \note The FemElement is returned with read/write access
	/// \note Out of range femElementId will raise an exception
	std::shared_ptr<FemElement> getFemElement(size_t femElementId);

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

	/// Determines whether the associated coordinate is valid
	/// \param coordinate Coordinate to check
	/// \return True if coordinate is valid
	bool isValidCoordinate(const SurgSim::DataStructures::IndexedLocalCoordinate &coordinate) const;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt) override;

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	/// \note This method will update all FemElement with the final state
	/// \note and potentially deactivate/reset the representation if necessary.
	void afterUpdate(double dt) override;

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	SurgSim::Math::Vector& computeF(const SurgSim::Math::OdeState& state) override;

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	const SurgSim::Math::Matrix& computeM(const SurgSim::Math::OdeState& state) override;

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	const SurgSim::Math::Matrix& computeD(const SurgSim::Math::OdeState& state) override;

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	const SurgSim::Math::Matrix& computeK(const SurgSim::Math::OdeState& state) override;

	/// Evaluation of f(x,v), M(x,v), D = -df/dv(x,v), K = -df/dx(x,v)
	/// When all the terms are needed, this method can perform optimization in evaluating everything together
	/// \param state (x, v) the current position and velocity to evaluate the various terms with
	/// \param[out] f The RHS f(x,v)
	/// \param[out] M The matrix M(x,v)
	/// \param[out] D The matrix D = -df/dv(x,v)
	/// \param[out] K The matrix K = -df/dx(x,v)
	/// \note Returns pointers, the internal data will remain unchanged until the next call to computeFMDK() or
	/// \note computeF(), computeM(), computeD(), computeK()
	void computeFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector** f, SurgSim::Math::Matrix** M,
			SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K) override;

protected:
	/// Adds the Rayleigh damping forces
	/// \param[in,out] f The force vector to cumulate the Rayleigh damping force into
	/// \param state The state vector containing positions and velocities
	/// \param useGlobalMassMatrix, useGlobalStiffnessMatrix True indicates that the global mass and stiffness matrices
	///        should be used (F = -c.M.v - d.K.v)
	/// \param scale A scaling factor to apply on the damping force
	/// \note Damping matrix D = c.M + d.K (Rayleigh damping definition)
	/// \note F = - D.v = -c.M.v - d.K.v
	/// \note If {useGlobalMassMatrix | useGlobalStiffnessMatrix} is True, {M | K} will be used
	/// \note If {useGlobalMassMatrix | useGlobalStiffnessMatrix} is False
	/// \note    the {mass|stiffness} component will be computed FemElement by FemElement
	void addRayleighDampingForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state,
		bool useGlobalMassMatrix = false, bool useGlobalStiffnessMatrix = false, double scale = 1.0);

	/// Adds the FemElements forces to f (given a state)
	/// \param[in,out] f The force vector to cumulate the FemElements forces into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the FemElements forces with
	void addFemElementsForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state, double scale = 1.0);

	/// Adds the gravity force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the gravity force into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the gravity force with
	/// \note This method does not do anything if gravity is disabled
	void addGravityForce(SurgSim::Math::Vector *f, const SurgSim::Math::OdeState& state, double scale = 1.0);

	bool doInitialize() override;

	/// Useful information per node
	std::vector<double> m_massPerNode; ///< Useful in setting up the gravity force F=mg

	/// Filename for loading the fem representation.
	std::string m_filename;

private:
	/// To be implemented by derived classes.
	/// \return The delegate to load the corresponding derived class.
	virtual std::shared_ptr<FemPlyReaderDelegate> getDelegate() = 0;

	/// FemElements
	std::vector<std::shared_ptr<FemElement>> m_femElements;

	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, M = mass, K = stiffness
	struct {
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMREPRESENTATION_H
